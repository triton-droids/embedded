"""Balance (the hand-over between get-up and walking): from just-stood-up states (the get-up policy's end states,
optionally from states_file) or perturbed stances, reach the walking stance and the hand-over check of
GetUpPlanner (torso within 0.25 rad of vertical, both feet > 40 N, base speed < 0.15 m/s, leg joints within
0.25 rad of the stance, soles within 10 deg of flat) and hold it for hold_steps. Falling ends the episode.

Start states (make_start): states_frac from states_file ('getup_end'), easy_frac near the balanced stance
('easy', training curriculum only: eval keeps easy_frac=0), the rest perturbed stances ('stance'). Stances are
placed with the lowest sole corner on the floor (no drop) and the base pitched so the soles are about flat: at
q_nominal with an upright base the soles are 11.5 deg off flat (hip1 + knee + ankle = -0.2 rad), so the
balanced stance has the pelvis pitched ~0.2 rad (or hip1 opened by up to 0.2 rad).

Reward: success ends the episode with a bonus worth more than standing on (bonus > max step reward / (1 - 0.99)),
so holding the hand-over check is never worse than avoiding it. The step reward is a soft version of the
check (each condition a wide + narrow kernel, combined as mean and product), plus potential-based shaping on
the same score, minus realism penalties (rl_tasks.common).

Braking from moving, tilted get-up end states is this task's job. fall_up=0.6 and fall_z=0.38 set the
upright / height fall thresholds; non-foot contact always counts as a fall. brake_w=0.0 optionally adds
brake_w * (phi_now - phi_prev), with phi = -(min(base speed, 1.5) + torso tilt in radians), starting on the
second step. Info includes first-step start_speed / start_tilt and brake_s: elapsed seconds when speed
< 0.3 m/s and tilt < 0.3 rad first hold for 5 consecutive steps (NaN until then)."""
import math

import numpy as np

from .common import realism_penalty

UP_MIN = math.cos(0.25)


def kernel(x, c):
    """1 at x = 0, ~0.54 at the threshold c, still a slope far out (x = 5c: ~0.09)."""
    x = max(0.0, float(x))
    return 0.5 * math.exp(-x / (3.0 * c)) + 0.5 * math.exp(-(x / c) ** 2)


class Task:
    history = 10
    command_dim = 0
    episode_s = 8.0
    priv_dim = 4

    def __init__(self, states_file='', states_frac=0.5, easy_frac=0.0, hold_steps=25, power_w=1e-4,
                 bonus=250.0, shaping=2.0, stance_depth=(0.3, 1.6), stance_noise=0.12, stance_vel=0.25, states_max_speed=0.0, states_max_tilt=0.0,
                 fall_up=0.6, fall_z=0.38, brake_w=0.0):
        self.fall_up, self.fall_z, self.brake_w = float(fall_up), float(fall_z), float(brake_w)
        self.states = dict(np.load(states_file)) if states_file else None
        if self.states is not None and (states_max_speed > 0 or states_max_tilt > 0):
            # Curriculum: keep only the calmer end states (base speed m/s / torso tilt rad below the limits).
            q, v = self.states['qpos'], self.states['qvel']
            keep = np.ones(len(q), bool)
            if states_max_speed > 0:
                keep &= np.linalg.norm(v[:, 0:3], axis=1) < float(states_max_speed)
            if states_max_tilt > 0:
                keep &= 2.0 * np.arcsin(np.clip(np.hypot(q[:, 4], q[:, 5]), 0, 1)) < float(states_max_tilt)
            print(f'balance: {int(keep.sum())}/{len(keep)} start states kept', flush=True)
            if keep.any():
                self.states = {k: np.asarray(x)[keep] for k, x in self.states.items()}
        self.states_frac, self.easy_frac = float(states_frac), float(easy_frac)
        self.hold_steps, self.power_w = int(hold_steps), float(power_w)
        self.bonus, self.shaping = float(bonus), float(shaping)
        self.stance_depth, self.stance_noise, self.stance_vel = tuple(stance_depth), float(stance_noise), \
            float(stance_vel)

    # ------------------------------------------------------------------ start states
    def _place(self, env, i, q, pitch, roll, yaw):
        """Joints q, base orientation yaw * roll(x) * pitch... with the lowest sole-box corner 2 mm above the floor."""
        s, m, d = env.sims[i], env.sims[i].model, env.sims[i].data
        mj = env.mj
        qy = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        qp = np.array([math.cos(pitch / 2), math.sin(pitch / 2), 0, 0])   # lateral axis of the model is x
        qr = np.array([math.cos(roll / 2), 0, math.sin(roll / 2), 0])
        tmp, quat = np.zeros(4), np.zeros(4)
        mj.mju_mulQuat(tmp, qy, qp)
        mj.mju_mulQuat(quat, tmp, qr)
        d.qpos[3:7] = quat
        d.qpos[s.qpos] = q
        s.release()
        mj.mj_forward(m, d)
        low = min(self._lowest(m, d, g) for g in s.foot_geoms)
        d.qpos[2] += 0.002 - low
        mj.mj_forward(m, d)

    @staticmethod
    def _lowest(m, d, g):
        if m.geom_type[g] == 6:   # box: exact lowest corner
            R = d.geom_xmat[g].reshape(3, 3)
            return float(d.geom_xpos[g, 2] - np.abs(R[2] * m.geom_size[g]).sum())
        return float(d.geom_xpos[g, 2] - m.geom_rbound[g])

    def _settle(self, env, i, q, steps=4):
        s = env.sims[i]
        for _ in range(steps):
            env.apply_mit(s, env.planner, env.Command(q, env.zero, env.planner.kp, env.planner.kd, env.zero))
            env.mj.mj_step(s.model, s.data)

    def make_start(self, env, i):
        u = env.rng.random()
        d = env.sims[i].data
        yaw = env.rng.uniform(-math.pi, math.pi)
        if self.states is not None and u < self.states_frac:
            env.from_states(i, self.states)
            return 'getup_end'
        if u < (self.states_frac if self.states is not None else 0.0) + self.easy_frac:
            q = env.q_nom.copy()
            q = np.clip(q + env.rng.normal(0, 0.04, len(q)), env.planner.lo, env.planner.hi)
            self._place(env, i, q, 0.2 + env.rng.normal(0, 0.04), env.rng.normal(0, 0.03), yaw)
            self._settle(env, i, q)
            d.qvel[:3] += env.rng.normal(0, 0.05, 3)
            return 'easy'
        # Perturbed stances, straight-legged to deep, soles roughly flat, with some base motion.
        depth = env.rng.uniform(*self.stance_depth)
        q = env.q_nom.copy()
        q[env.leg] *= depth
        q = np.clip(q + env.rng.normal(0, self.stance_noise, len(q)), env.planner.lo, env.planner.hi)
        self._place(env, i, q, 0.2 * depth + env.rng.normal(0, 0.08), env.rng.normal(0, 0.05), yaw)
        self._settle(env, i, q)
        d.qvel[:3] += env.rng.normal(0, self.stance_vel, 3)
        d.qvel[3:6] += env.rng.normal(0, 0.3, 3)
        return 'stance'

    def on_reset(self, env, i):
        env.state[i]['ready'] = 0
        env.state[i]['phi'] = None
        env.state[i].update(brake_phi=None, brake_hold=0, brake_s=math.nan, steps=0,
                            start_speed=math.nan, start_tilt=math.nan)

    # ------------------------------------------------------------------ privileged / reward
    def _com_offset(self, env, i):
        """Horizontal CoM minus the mean sole-box centre, in the base's yaw frame (m)."""
        s, d = env.sims[i], env.sims[i].data
        feet = np.mean([d.geom_xpos[g] for g in s.foot_geoms], 0)
        off = d.subtree_com[1][:2] - feet[:2]
        w, x, y, z = d.qpos[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, sn = math.cos(yaw), math.sin(yaw)
        return c * off[0] + sn * off[1], -sn * off[0] + c * off[1]

    def priv(self, env, i):
        dx, dy = self._com_offset(env, i)
        return [env.state[i].get('ready', 0) / self.hold_steps, 10 * dx, 10 * dy, env.state[i].get('score', 0.0)]

    def score(self, meas):
        f = meas['forces']
        tilt = math.acos(min(1.0, max(-1.0, meas['up'])))
        ks = [kernel(tilt - 0.15, 0.1),                       # torso: 0.25 rad allowed, aim inside
              kernel(40.0 - min(f) + 10.0, 30.0),              # both feet loaded (> 40 N, aim 50 N)
              kernel(meas['speed'] - 0.05, 0.1),               # base speed < 0.15 m/s
              kernel(meas['err_max'] - 0.1, 0.15),             # leg joints within 0.25 rad of the stance
              kernel(max(meas['tilts']) - 4.0, 6.0)]           # soles < 10 deg off flat
        return 0.5 * float(np.mean(ks)) + 0.5 * float(np.prod(ks)), ks

    def reward(self, env, i, meas, action):
        st = env.state[i]
        tilt = math.acos(min(1.0, max(-1.0, meas['up'])))
        st['steps'] += 1
        if st['steps'] == 1:
            st['start_speed'], st['start_tilt'] = float(meas['speed']), tilt
        brake_phi = -(min(meas['speed'], 1.5) + tilt)
        brake = 0.0 if st['brake_phi'] is None else self.brake_w * (brake_phi - st['brake_phi'])
        st['brake_phi'] = brake_phi
        st['brake_hold'] = st['brake_hold'] + 1 if meas['speed'] < 0.3 and tilt < 0.3 else 0
        if st['brake_hold'] >= 5 and math.isnan(st['brake_s']):
            st['brake_s'] = st['steps'] * 0.02
        info = dict(start_speed=st['start_speed'], start_tilt=st['start_tilt'], brake_s=st['brake_s'])
        fell = meas['other'] or meas['z'] < self.fall_z or meas['up'] < self.fall_up
        if fell:
            return -5.0, True, False, dict(fell=True, **info)
        f = meas['forces']
        ready_like = (meas['up'] > UP_MIN and min(f) > 40.0 and meas['speed'] < 0.15
                      and meas['err_max'] < 0.25 and max(meas['tilts']) < 10.0)
        st['ready'] = st['ready'] + 1 if ready_like else 0
        sc, ks = self.score(meas)
        st['score'] = sc
        phi = self.shaping * sc
        shaping = 0.0 if st['phi'] is None else 0.99 * phi - st['phi']
        st['phi'] = phi
        r = (0.2 + 0.6 * sc + 0.2 * math.exp(-meas['leg_err'] / 0.05) + 0.5 * ready_like
             + 0.5 * min(st['ready'], self.hold_steps) / self.hold_steps + shaping + brake
             - 0.0002 * float(np.sum(meas['dq'] ** 2)) - 0.01 * float(np.sum(meas['ang_body'] ** 2))
             - realism_penalty(meas, power_w=self.power_w))
        success = st['ready'] >= self.hold_steps
        if success:
            r += self.bonus
        return r, success, success, dict(fell=False, score=sc, err_max=meas['err_max'],
                                         sole_tilt=max(meas['tilts']), speed=meas['speed'], min_f=min(f), **info)
