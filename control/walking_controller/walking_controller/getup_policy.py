"""Get-up student policy (distilled from the get-up MPC, tools/getup_demo.py + tools/getup_bc.py).

Observation, one 50 Hz control tick: the last `history` frames of
  [base angular velocity (body, 3), gravity direction (body, 3), joint_pos - q_nominal (n), joint_vel (n)]
oldest first, then the previous action (n), then (time_input) the time since the get-up started / 10 s,
clipped to [0, 2], then (command_dim) a command such as the walking velocity (vx, vy, wz). Action a: joint target = q_nominal + scale * a, tracked with the MPC's MIT gains.
No MuJoCo: the policy file (npz) carries the normalisation, MLP weights and the layout above.
"""
from __future__ import annotations

import numpy as np

CONTROL_DT = 0.02


def quat_rotate_inverse(quat_wxyz, vec):
    """Rotate a world vector into the body frame of quaternion (w, x, y, z)."""
    w, x, y, z = quat_wxyz
    q = np.array([x, y, z])
    v = np.asarray(vec, float)
    t = 2.0 * np.cross(q, v)
    return v - w * t + np.cross(q, t)


def frame(ang_body, quat_wxyz, joint_pos, joint_vel, q_nominal):
    grav = quat_rotate_inverse(np.asarray(quat_wxyz, float) / np.linalg.norm(quat_wxyz), [0.0, 0.0, -1.0])
    return np.concatenate([ang_body, grav, np.asarray(joint_pos) - q_nominal, joint_vel])


def observation(frames, previous_action, elapsed_s=None, command=None):
    parts = [np.asarray(frames).ravel(), previous_action]
    if elapsed_s is not None:
        parts.append([np.clip(elapsed_s / 10.0, 0.0, 2.0)])
    if command is not None:
        parts.append(np.asarray(command, float))
    return np.concatenate(parts)


class GetUpPolicy:
    """Loads the npz from getup_bc.py; step() keeps the frame history and the previous action."""

    def __init__(self, path):
        z = np.load(path)
        self.mean, self.std = z['obs_mean'], z['obs_std']
        n = int(z['layers'])
        self.w = [z[f'w{i}'] for i in range(n)]
        self.b = [z[f'b{i}'] for i in range(n)]
        self.q_nominal = np.asarray(z['q_nominal'], float)
        self.scale = float(z['scale'])
        self.history = int(z['history'])
        self.time_input = bool(z['time_input']) if 'time_input' in z.files else False
        self.command_dim = int(z['command_dim']) if 'command_dim' in z.files else 0
        self.task = str(z['task']) if 'task' in z.files else 'getup'
        # RL training clips the network output to +-3.5 (rad, before scale); keep it for deployment.
        self.action_clip = float(z['action_clip']) if 'action_clip' in z.files else 3.5
        self.joint_names = [str(s) for s in z['joint_names']]
        self.reset()

    def reset(self):
        self.frames, self.prev = None, np.zeros(len(self.q_nominal))

    def __call__(self, obs):
        x = (obs - self.mean) / self.std
        for i, (w, b) in enumerate(zip(self.w, self.b)):
            x = x @ w.T + b
            if i < len(self.w) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return x

    def step(self, current_frame, elapsed_s, command=None):
        """Push one frame, return (action, joint targets). command: required when command_dim > 0."""
        if self.frames is None:
            self.frames = [current_frame] * self.history
        self.frames = self.frames[1:] + [current_frame]
        if self.command_dim and (command is None or len(command) != self.command_dim):
            raise ValueError(f'this policy needs a command of length {self.command_dim}')
        action = self(observation(self.frames, self.prev, elapsed_s if self.time_input else None,
                                  command if self.command_dim else None))
        # As in training (tools/rl_env.py TaskEnv.step): the clipped action is both applied and fed back.
        action = np.clip(action, -self.action_clip, self.action_clip)
        self.prev = action
        return action, self.q_nominal + self.scale * action
