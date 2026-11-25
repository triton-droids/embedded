"""Walk: track a velocity command (forward, left in the heading frame, yaw rate), resampled every resample_s,
starting from the walking stance (or saved balance end states). A command of exactly zero means stand still on
both feet. Falling (non-foot floor contact, z < 0.35 m, torso tilted > ~50 deg) ends the episode.

Command (observation tail, command_dim = 5): [v_forward, v_left, wz, sin(2 pi phase), cos(2 pi phase)]. The last
two are a gait clock of period `period` that the controller advances while walking; both are zero for a zero
command (stand). Left foot in stance for phase in [0, stance), right foot half a period later.

Frames: the model faces +y of its base frame (left_* on -x), so the forward velocity is rl_env's meas['vy'] and
the leftward one is -meas['vx'].

Episode info (rl_eval): vx_err = mean over command segments (first 1 s of each skipped) of |mean forward velocity -
command|, vx_err_inst = mean instantaneous |v_forward - command|, plus gait statistics (touchdowns, air time,
flight fraction, peak speed ratio, power, steps taken while standing).

Optional ref_file supplies phase-centred absolute q_ref poses from walk_ref.py. While moving with an active
clock, add ref_w * exp(-sum(joint_error**2) / ref_sigma), before advancing the clock. ref_sigma is in rad^2;
ref_legs_only selects env.leg instead of all joints. ref_match reports the unweighted moving-step mean
(zero if no active steps). With no reference file the legacy reward and episode info are unchanged.
"""
import math

import numpy as np

from .common import realism_penalty


def interpolate_reference(phase, phases, q_ref):
    """Linearly interpolate absolute poses, including across the cycle's 1-to-0 seam."""
    p = float(phase) % 1.0
    right = int(np.searchsorted(phases, p, side='right'))
    left = right - 1
    lo = phases[left] if left >= 0 else phases[-1] - 1.0
    hi = phases[right] if right < len(phases) else phases[0] + 1.0
    weight = (p - lo) / (hi - lo)
    return (1.0 - weight) * q_ref[left] + weight * q_ref[right % len(phases)]


class Task:
    history = 5
    command_dim = 5
    episode_s = 16.0
    priv_dim = 6

    def __init__(self, states_file='', vx=(0.0, 0.3), vy=(-0.05, 0.05), wz=(-0.15, 0.15), zero_frac=0.25,
                 resample_s=5.0, power_w=6e-4, period=0.7, stance=0.6, swing_h=0.06, base_h=0.565,
                 episode_s=16.0, push=0.0, ref_file='', ref_w=0.0, ref_sigma=0.25, ref_legs_only=True):
        self.states = dict(np.load(states_file)) if states_file else None
        self.ranges = (tuple(vx), tuple(vy), tuple(wz))
        self.zero_frac, self.resample = float(zero_frac), int(round(float(resample_s) / 0.02))
        self.power_w = float(power_w)
        self.period, self.stance_frac, self.swing_h, self.base_h = float(period), float(stance), float(swing_h), \
            float(base_h)
        self.episode_s = float(episode_s)
        self.push = float(push)   # peak push force (N) of random 0.2 s shoves on the base, 0 = none
        self.ref = None
        self.ref_w, self.ref_sigma, self.ref_legs_only = float(ref_w), float(ref_sigma), bool(ref_legs_only)
        if ref_file:
            if not math.isfinite(self.ref_sigma) or self.ref_sigma <= 0 or not math.isfinite(self.ref_w):
                raise ValueError('require finite ref_w and positive finite ref_sigma')
            with np.load(ref_file, allow_pickle=False) as ref:
                self.ref = {key: ref[key].copy() for key in ('phase', 'q_ref', 'joint_names')}
                for key, expected in (('period', self.period), ('stance', self.stance_frac)):
                    if float(ref[key]) != expected:
                        raise ValueError(f'reference {key} must equal task {key} ({expected})')
            ph, q = self.ref['phase'], self.ref['q_ref']
            if (ph.ndim != 1 or len(ph) == 0 or not np.isfinite(ph).all()
                    or np.any(np.diff(ph) <= 0) or np.any((ph < 0) | (ph >= 1))
                    or q.shape != (len(ph), 18) or not np.isfinite(q).all()):
                raise ValueError('invalid reference phase or q_ref arrays')

    def _check_reference(self, env):
        if self.ref is not None and not np.array_equal(self.ref['joint_names'], env.names):
            raise ValueError('reference joint_names must match environment joint order')

    def _reference_reward(self, env, i, meas):
        """Accumulate unweighted match statistics for active moving clocks only."""
        st = env.state[i]
        if self.ref is None or not st['moving'] or np.abs(env.command[i, 3:5]).sum() <= 0.5:
            return 0.0
        q_r = interpolate_reference(st['phase'], self.ref['phase'], self.ref['q_ref'])
        error = (meas['q'] - q_r)[env.leg if self.ref_legs_only else slice(None)]
        match = math.exp(-float(np.sum(error ** 2)) / self.ref_sigma)
        st['ref_sum'] += match
        st['ref_n'] += 1
        return self.ref_w * match

    # ------------------------------------------------------------------ start / commands
    def make_start(self, env, i):
        self._check_reference(env)
        if self.states is not None and env.rng.random() < 0.5:
            env.from_states(i, self.states, noise=0.01)
            return 'balance_end'
        # env.stance() places the soles by their bounding spheres (~8 cm in the air): put them on the floor.
        env.stance(i, depth=1.0, noise=0.02, settle_steps=0)
        s, m, d = env.sims[i], env.sims[i].model, env.sims[i].data
        low = min(d.geom_xpos[g, 2] - np.abs(d.geom_xmat[g].reshape(3, 3)[2, :]) @ m.geom_size[g]
                  for g in s.foot_geoms)
        d.qpos[2] += 0.002 - low
        env.mj.mj_forward(m, d)
        q = d.qpos[s.qpos].copy()
        for _ in range(2):
            env.apply_mit(s, env.planner, env.Command(q, env.zero, env.planner.kp, env.planner.kd, env.zero))
            env.mj.mj_step(m, d)
        return 'stance'

    def _sample(self, env, i):
        st = env.state[i]
        was_moving = st.get('moving', False)
        if env.rng.random() < self.zero_frac:
            env.command[i, :3] = 0.0
        else:
            c = [env.rng.uniform(*r) for r in self.ranges]
            lo, hi = self.ranges[0]
            c[0] = lo + (hi - lo) * env.rng.random() ** 1.5   # more weight on slow forward speeds
            env.command[i, :3] = c
        st['moving'] = bool(np.any(env.command[i, :3] != 0.0))
        if st['moving'] and not was_moving:
            st['phase'] = float(env.rng.choice([0.0, 0.5]))
        st['seg_v'], st['seg_n'], st['seg_t'] = 0.0, 0, 0
        self._clock(env, i)

    def _clock(self, env, i):
        st = env.state[i]
        if st['moving']:
            a = 2 * math.pi * st['phase']
            env.command[i, 3:5] = [math.sin(a), math.cos(a)]
        else:
            env.command[i, 3:5] = 0.0

    def on_reset(self, env, i):
        self._check_reference(env)
        if self.ref is not None:
            env.state[i].update(ref_sum=0.0, ref_n=0)
        d = env.sims[i].data
        sites = env.sole_sites[i]
        env.state[i].update(moving=False, phase=0.0, contact=[True, True], air=[0.0, 0.0],
                            feet=d.site_xpos[sites].copy(), dq=d.qvel[env.sims[i].qvel].copy(),
                            seg_errs=[], inst=0.0, n=0, touch=[0, 0], air_sum=0.0, air_n=0, flight=0,
                            ratio_max=0.0, power=0.0, stand_lift=0, stand_n=0, push_t=0)
        self._sample(env, i)

    def priv(self, env, i):
        st = env.state[i]
        air = st.get('air', [0.0, 0.0])
        return [air[0], air[1], float(st.get('moving', False)), float(st.get('phase', 0.0)),
                float(env.command[i, 0]), float(env.command[i, 2])]

    def _close_segment(self, st, cmd):
        if st['seg_n'] > 0:
            st['seg_errs'].append(abs(st['seg_v'] / st['seg_n'] - cmd))

    # ------------------------------------------------------------------ reward
    def reward(self, env, i, meas, action):
        st = env.state[i]
        s, d = env.sims[i], env.sims[i].data
        if env.t[i] > 0 and env.t[i] % self.resample == 0:
            self._close_segment(st, env.command[i, 0])
            self._sample(env, i)
        if self.push > 0:
            self._push(env, i)
        ref_reward = self._reference_reward(env, i, meas) if self.ref is not None else 0.0
        fell = meas['other'] or meas['z'] < 0.35 or meas['up'] < 0.65
        if fell:
            return -10.0 + ref_reward, True, False, self._info(st, True, env.command[i, 0])
        cf, cl, cw = env.command[i, :3]
        moving = st['moving']
        v_fwd, v_left = meas['vy'], -meas['vx']
        yaw = meas['yaw']
        ex = np.array([math.cos(yaw), math.sin(yaw)])     # base x (robot's right) in the world
        ey = np.array([-math.sin(yaw), math.cos(yaw)])    # robot's forward
        feet = d.site_xpos[env.sole_sites[i]]
        fvel = (feet - st['feet']) / 0.02
        st['feet'] = feet.copy()
        dq = meas['dq']
        ddq = (dq - st['dq']) / 0.02
        st['dq'] = dq.copy()
        q_err = meas['q'] - env.q_nom

        err_v = (v_fwd - cf) ** 2 + (v_left - cl) ** 2
        # A sharp peak plus a linear pull: standing still must not pay off when a velocity is commanded.
        r = (1.5 * math.exp(-err_v / 0.04) + 1.5 * math.exp(-err_v / 0.005) - 1.5 * math.sqrt(err_v)
             + 0.75 * math.exp(-(meas['yaw_rate'] - cw) ** 2 / 0.1) + 0.5
             - 2.0 * meas['vz'] ** 2 - 0.05 * float(np.sum(meas['ang_body'][:2] ** 2))
             - 5.0 * (1.0 - meas['up']) - 20.0 * (meas['z'] - self.base_h) ** 2
             - 5e-4 * float(np.sum(dq ** 2)) - 1e-6 * float(np.sum(ddq ** 2))
             - realism_penalty(meas, power_w=self.power_w))
        # Hip roll / yaw near the stance (hip2, thigh), arms near their pose.
        legs = env.names
        hip = [k for k, n in enumerate(legs) if 'hip2' in n or 'thigh' in n]
        arms = [k for k, n in enumerate(legs) if 'shoulder' in n or 'elbow' in n]
        r -= 1.5 * float(np.sum(q_err[hip] ** 2)) + 0.3 * float(np.sum(q_err[arms] ** 2))
        # Feet: lateral spacing (right foot on +x of the base), no slipping while in contact.
        sep = float((feet[1, :2] - feet[0, :2]) @ ex)
        r -= 100.0 * max(0.0, 0.26 - sep) ** 2 + 10.0 * max(0.0, sep - 0.5) ** 2
        contact = (meas['left'], meas['right'])
        for k in range(2):
            if contact[k]:
                r -= 0.5 * float(np.sum(fvel[k, :2] ** 2))
        # Gait clock: contact pattern and swing height (phase-shaped target), or stand on both feet.
        if self.ref is not None:
            r += ref_reward
        if moving:
            for k in range(2):
                ph = (st['phase'] + 0.5 * k) % 1.0
                stance_des = ph < self.stance_frac
                if stance_des:
                    r += 0.8 * contact[k]
                else:
                    # swing credit only for a foot that is really lifted (contact flicker with the sole on the
                    # floor, the shuffle the policy found, earns nothing)
                    prog_h = self.swing_h * math.sin(math.pi * (ph - self.stance_frac) / (1.0 - self.stance_frac))
                    r += 0.8 * ((not contact[k]) and feet[k, 2] > 0.5 * prog_h and feet[k, 2] > 0.015)
                if not stance_des:
                    prog = (ph - self.stance_frac) / (1.0 - self.stance_frac)
                    target = self.swing_h * math.sin(math.pi * prog)
                    r += 0.8 * math.exp(-(feet[k, 2] - target) ** 2 / 0.0016)
            st['phase'] = (st['phase'] + 0.02 / self.period) % 1.0
        else:
            r += 0.4 * contact[0] + 0.4 * contact[1]
            r -= 1.0 * float(np.sum(fvel[:, :2] ** 2)) + 0.5 * float(np.mean(q_err[env.leg] ** 2))
            st['stand_n'] += 1
        self._clock(env, i)
        # Statistics.
        for k in range(2):
            if contact[k] and not st['contact'][k]:
                st['touch'][k] += 1
                if moving and st['air'][k] < 0.15:
                    r -= 5.0      # chattering / shuffling touchdowns: a real step keeps the foot up >= 0.15 s
                st['air_sum'] += st['air'][k]
                st['air_n'] += 1
                if not moving:
                    st['stand_lift'] += 1
                st['air'][k] = 0.0
            elif not contact[k]:
                st['air'][k] += 0.02
            st['contact'][k] = contact[k]
        st['flight'] += int(meas['airborne'])
        st['ratio_max'] = max(st['ratio_max'], meas['speed_ratio'])
        st['power'] += meas['power']
        st['n'] += 1
        st['inst'] += abs(v_fwd - cf)
        st['seg_t'] += 1
        if st['seg_t'] > 50:
            st['seg_v'] += v_fwd
            st['seg_n'] += 1
        success = env.t[i] >= env.steps_max - 1
        info = self._info(st, False, cf) if success else {}
        return r, success, success, info

    def _push(self, env, i):
        st, d = env.state[i], env.sims[i].data
        body = 1  # the base
        if st['push_t'] > 0:
            st['push_t'] -= 1
            if st['push_t'] == 0:
                d.xfrc_applied[body, :3] = 0.0
        elif env.rng.random() < 0.02 * 0.25:   # about every 4 s
            ang = env.rng.uniform(-math.pi, math.pi)
            f = env.rng.uniform(0.3, 1.0) * self.push
            d.xfrc_applied[body, :3] = [f * math.cos(ang), f * math.sin(ang), 0.0]
            st['push_t'] = 10

    def _info(self, st, fell, cmd=None):
        if cmd is not None:
            self._close_segment(st, cmd)
        n = max(1, st['n'])
        # Falls / short segments: fall back to the instantaneous error so the evaluation mean is never NaN.
        info = dict(fell=fell, vx_err=float(np.mean(st['seg_errs'])) if st['seg_errs'] else st['inst'] / n,
                    vx_err_inst=st['inst'] / n, touchdowns=sum(st['touch']), air_mean=st['air_sum'] / max(1, st['air_n']),
                    flight_frac=st['flight'] / n, ratio_max=st['ratio_max'], power_mean=st['power'] / n,
                    stand_lift=st['stand_lift'], stand_steps=st['stand_n'])
        if self.ref is not None:
            info['ref_match'] = st['ref_sum'] / max(1, st['ref_n'])
        return info
