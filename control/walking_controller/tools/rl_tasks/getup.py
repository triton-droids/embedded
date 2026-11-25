"""Get-up: from lying (random, prone, supine) or a deep squat until standing on the feet only for hold_steps:
z > 0.5 m, upright > max(0.85, cos(max_tilt)), base speed < max_speed.
Defaults max_speed=math.inf and max_tilt=math.acos(0.85) retain the lenient upright > 0.85 check.
Tighter limits optionally require a calm hand-over. Success ends the episode; balance handles braking.

Optional saved starts: states_file='' optionally loads saved qpos/qvel arrays once from an npz; states_frac=0.0
selects the share of starts restored from them with joint noise 0.02 (label 'getup_end'). These starts are
already risen and earn no rise bonus. After rising, non-foot floor contact or z < 0.35 m ends the episode
with fall_penalty=20.0 subtracted, unless the success check passed. Info reports fell for this termination.

Real-robot feasibility: the motors have torque-speed limits in the simulation, but a policy can still throw the body
(flight, fast vertical motion, kilowatts of mechanical power). On top of the shared realism penalty the get-up
penalises flight harder (and fails the episode after a few airborne steps), mechanical power above a soft budget,
|vz| and joint speed near their limits. Each episode reports, in its done info: airborne_steps, max_power (W),
max_vz (m/s), max_speed_ratio (joint speed / no-load speed, ankles excluded) and rise_s (time to success)."""
import math

import numpy as np

from .common import flat_factor, realism_penalty


class Task:
    history = 10
    command_dim = 0
    episode_s = 15.0
    priv_dim = 1

    def __init__(self, crouch_frac=0.2, hold_steps=25, airborne=20.0, power_w=2e-4, power_budget=1500.0,
                 power_over=6e-3, vz_limit=0.6, vz_w=12.0, speed_limit=0.6, speed_w=20.0, air_fail=3,
                 air_fail_penalty=40.0, success_bonus=300.0, phi_scale=10.0, gamma=0.995,
                 rate_w=0.05, max_speed=math.inf, max_tilt=math.acos(0.85), calm_w=0.7, rise_bonus_frac=0.5, stand_reward=0.0,
                 states_file='', states_frac=0.0, fall_penalty=20.0):
        self.states = None
        if states_file:
            with np.load(states_file) as states:
                self.states = dict(states)
        self.states_frac, self.fall_penalty = float(states_frac), float(fall_penalty)
        self.crouch_frac, self.hold_steps = float(crouch_frac), int(hold_steps)
        self.airborne, self.power_w = float(airborne), float(power_w)
        self.power_budget, self.power_over = float(power_budget), float(power_over)
        self.vz_limit, self.vz_w = float(vz_limit), float(vz_w)
        self.speed_limit, self.speed_w = float(speed_limit), float(speed_w)
        self.air_fail, self.air_fail_penalty = int(air_fail), float(air_fail_penalty)
        self.success_bonus, self.phi_scale, self.gamma = float(success_bonus), float(phi_scale), float(gamma)
        self.rate_w = float(rate_w)
        # Optional calm limits for the rise check; defaults preserve the lenient get-up hand-over.
        self.max_speed, self.min_up = float(max_speed), max(0.85, math.cos(float(max_tilt)))
        self.calm_w = float(calm_w)
        # Two-part bonus: rise_bonus_frac of the success bonus when the old (lenient) stand check first holds, the
        # rest at the calm hand-over check (success, episode ends). With the whole bonus behind the harder calm
        # check, success became rare and the heavy realism penalties made not getting up the best policy.
        self.rise_bonus_frac = float(rise_bonus_frac)
        # Per-step reward while risen (lenient stand check), so standing while not yet calm pays more than lying
        # (the episode now goes on after rising, and holding a stance costs power / joint-speed penalties).
        self.stand_reward = float(stand_reward)

    def make_start(self, env, i):
        if self.states is not None and self.states_frac > 0.0 and env.rng.random() < self.states_frac:
            env.from_states(i, self.states, noise=0.02)
            return 'getup_end'
        if env.rng.random() < self.crouch_frac:
            env.stance(i, depth=env.rng.uniform(1.2, 1.8), noise=0.05, settle_steps=60)
            return 'crouch'
        return env.lying(i)   # 'random', 'prone' or 'supine'

    def on_reset(self, env, i):
        env.state[i].update(stand=0, rise=0, rose=env.label[i] == 'getup_end', air=0, air_steps=0, max_power=0.0, max_vz=0.0,
                            max_ratio=0.0, steps=0)

    def priv(self, env, i):
        return [env.state[i].get('stand', 0) / self.hold_steps]

    def potential(self, meas, standing):
        """Progress towards standing; the reward pays its (discounted) increase, so hovering just short of the
        rise check earns nothing (a plain per-step height reward made not succeeding pay: success ends the
        episode)."""
        h = min(max(meas['z'], 0.0), 0.62) / 0.62
        flat = flat_factor(meas)
        # Calm on the feet: pays slowing down once up, so the hand-over gets a robot it can balance.
        calm = meas['feet_only'] * (meas['z'] > 0.45) * max(0.0, 1.0 - meas['speed'] / 0.6) * max(0.0, meas['up'])
        return self.phi_scale * (0.3 * h + 1.0 * h ** 4 + 0.3 * max(meas['up'], 0.0)
                                 + 0.5 * meas['feet_only'] * (0.5 + 0.5 * flat) + self.calm_w * calm
                                 + 1.0 * standing * flat)

    def reward(self, env, i, meas, action):
        st = env.state[i]
        st['steps'] += 1
        air = bool(meas['airborne'])
        st['air'] = st['air'] + 1 if air else 0
        st['air_steps'] += air
        st['max_power'] = max(st['max_power'], meas['power'])
        st['max_vz'] = max(st['max_vz'], abs(meas['vz']))
        st['max_ratio'] = max(st['max_ratio'], meas['speed_ratio'])
        standing = (meas['feet_only'] and meas['z'] > 0.5 and meas['up'] > self.min_up
                    and meas['speed'] < self.max_speed)
        st['stand'] = st['stand'] + 1 if standing else 0
        risen = meas['feet_only'] and meas['z'] > 0.5 and meas['up'] > 0.85
        st['rise'] = st['rise'] + 1 if risen else 0
        phi = self.potential(meas, standing)
        prev = st.get('phi', phi)
        st['phi'] = phi
        r = (self.gamma * phi - prev
             - 0.0005 * float(np.sum(meas['dq'] ** 2)) - 0.01 * float(np.sum(meas['ang_body'] ** 2))
             - realism_penalty(meas, airborne=self.airborne, power_w=self.power_w))
        r -= self.power_over * max(0.0, meas['power'] - self.power_budget)
        r -= self.vz_w * max(0.0, abs(meas['vz']) - self.vz_limit)
        r -= self.speed_w * max(0.0, meas['speed_ratio'] - self.speed_limit)
        if risen:
            r += self.stand_reward
        if self.rate_w and st['steps'] > 1:  # env.prev is still the previous action here
            r -= self.rate_w * float(np.sum((action - env.prev[i]) ** 2))
        success = st['stand'] >= self.hold_steps
        done = success
        fell = False
        if not st['rose'] and st['rise'] >= self.hold_steps:
            st['rose'] = True
            r += self.rise_bonus_frac * self.success_bonus * (0.5 + 0.5 * flat_factor(meas))
        if success:
            r += (1.0 - self.rise_bonus_frac if st['rose'] else 1.0) * self.success_bonus * (0.5 + 0.5 * flat_factor(meas))
        elif st['rose'] and (meas['other'] or meas['z'] < 0.35):
            r -= self.fall_penalty
            done = fell = True
        elif self.air_fail and st['air'] >= self.air_fail:
            r -= self.air_fail_penalty
            done = True
        info = dict(rose=bool(st['rose']), airborne_steps=int(st['air_steps']), max_power=float(st['max_power']),
                    max_vz=float(st['max_vz']), max_speed_ratio=float(st['max_ratio']),
                    rise_s=float(st['steps'] * 0.02) if success else float('nan'),
                    air_fail=bool(done and not success and not fell), fell=bool(fell))
        return r, done, success, info
