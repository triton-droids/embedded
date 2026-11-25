"""RL walking policy (tools/rl_train.py --task walk) behind the walking controller's interface, so the ROS node
can use it in place of SteppingController (walk_policy parameter).

The policy runs at 50 Hz on the IMU-like base state and the joint encoders (getup_policy observation) plus the
velocity command (vx, vy forward / left in the heading frame, wz yaw rate) and, for policies with a 5-value command
(tools/rl_tasks/walk.py), a gait clock [sin(2 pi phase), cos(2 pi phase)] advanced by dt / period while moving and
zero (phase reset) for a zero command; its joint targets are held between
its ticks and tracked with the get-up planner's MIT gains. A torso tilt above max_tilt_rad is a fault, as in the
model-based controller (with fall recovery the node then starts the get-up).
"""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from .gait import ACTIVE, FAULT, HOLD
from .getup_policy import CONTROL_DT, GetUpPolicy, frame, quat_rotate_inverse
from .offline import Command


class PolicyWalkController:
    def __init__(self, policy_path, joint_names, kp, kd, lower, upper, max_tilt_rad=0.5,
                 max_speed=(0.3, 0.15, 0.6), period=0.7):
        self.policy = GetUpPolicy(policy_path)
        if self.policy.command_dim not in (3, 5):
            raise ValueError('walk_policy must be a walking policy (command of 3: vx, vy, wz, or 5 with a gait clock)')
        self.period, self.gait_phase, self._t_tick = float(period), 0.0, None
        if self.policy.joint_names != list(joint_names):
            raise ValueError('walk_policy joint order differs from the controller')
        self.q_nominal = self.policy.q_nominal
        self.kp, self.kd = np.asarray(kp, float), np.asarray(kd, float)
        self.lo, self.hi = np.asarray(lower, float), np.asarray(upper, float)
        self.max_tilt, self.max_speed = float(max_tilt_rad), np.asarray(max_speed, float)
        # Attributes the node reads from SteppingController (step length / turn are unused by the policy).
        self.p = SimpleNamespace(step_length_m=0.0, turn_rad=0.0, t_ds=0.25, t_ss=0.25)
        self.phase, self.fault, self.status, self.steps = HOLD, '', {}, 0
        self.stop_requested, self.holding = False, True
        self.command = np.zeros(3)
        self.measured = None          # (quat_wxyz, angular_world, joint_pos, joint_vel), set by the node
        self._q, self._next, self._t0 = None, -1.0, None

    # ------------------------------------------------------------------ commands
    def set_velocity(self, vx, wz, vy=0.0):
        self.command = np.clip([vx, vy, wz], -self.max_speed, self.max_speed)

    def request_stop(self):
        self.stop_requested = True
        self.command[:] = 0.0

    def resume(self):
        self.stop_requested = False

    def request_kick(self, foot):
        self.status['kick'] = 'unsupported with walk_policy'

    def _policy_command(self, t):
        """The command the policy was trained with: velocity, plus the gait clock for 5-value policies."""
        if self.policy.command_dim == 3:
            return self.command.copy()
        moving = bool(np.any(self.command))
        if not moving:
            self.gait_phase = 0.0
        elif self._t_tick is not None:
            self.gait_phase = (self.gait_phase + (t - self._t_tick) / self.period) % 1.0
        self._t_tick = t
        if not moving:
            return np.zeros(5)
        a = 2.0 * np.pi * self.gait_phase
        return np.concatenate([self.command, [np.sin(a), np.cos(a)]])

    # ------------------------------------------------------------------ control
    def start_from_stand(self, t, q, v, forces):
        self.policy.reset()
        self._q, self._next, self._t0 = None, t, t
        self.gait_phase, self._t_tick = 0.0, None
        self.phase = ACTIVE

    def step(self, t, q, v, forces, base_held, age_s=0.0):
        if self.measured is None:
            self.fault, self.phase = 'no measured state', FAULT
            return None
        quat, ang_world, joints, rates = self.measured
        quat = np.asarray(quat, float) / np.linalg.norm(quat)
        tilt = float(np.arccos(np.clip(1 - 2 * (quat[1] ** 2 + quat[2] ** 2), -1, 1)))
        self.status = {'phase': self.phase, 'tilt': tilt, 'command': self.command.round(3).tolist(),
                       'gait_phase': round(self.gait_phase, 3) if self.policy.command_dim == 5 else None}
        zero = np.zeros(len(joints))
        if base_held:
            # Gantry: hold the stance; the policy starts when released.
            self.phase, self._q = HOLD, None
            return Command(self.q_nominal.copy(), zero, self.kp, self.kd, zero)
        if tilt > self.max_tilt:
            self.fault, self.phase = f'Torso tilt {tilt:.2f} rad', FAULT
            return None
        if self.phase == HOLD:
            self.start_from_stand(t, q, v, forces)
        if self._q is None or t + 1e-6 >= self._next:
            ang_body = quat_rotate_inverse(quat, ang_world)
            current = frame(ang_body, quat, joints, rates, self.q_nominal)
            _, target = self.policy.step(current, t - self._t0, self._policy_command(t))
            self._q = np.clip(target, self.lo, self.hi)
            self._next = max(self._next + CONTROL_DT, t)
        self.holding = not np.any(self.command)
        return Command(self._q, zero, self.kp, self.kd, zero)
