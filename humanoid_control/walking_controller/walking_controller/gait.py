"""Gait controller: gantry hold, landing, squat, then a balance activity.

Activities (``GaitParams.activity``):
- 'stand': hold the squat.
- 'sway':  double support, CoM moves sideways sinusoidally. Both feet roll
           on parallel edges (towards the side the CoM is on); this is the
           experiment that decides whether lateral weight transfer works.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .offline import Command
from .robot import FEET, RobotModel
from .wbc import FLAT, INNER, OUTER, WBCGains, WholeBodyController

HOLD, LANDING, SQUAT, ACTIVE, FAULT = 'hold', 'landing', 'squat', 'active', 'fault'
GRAVITY = 9.81


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return 10*s**3 - 15*s**4 + 6*s**5, 30*s**2 - 60*s**3 + 30*s**4, 60*s - 180*s**2 + 120*s**3


@dataclass
class GaitParams:
    dt: float = 0.005
    hold_kp: float = 40.0
    hold_kd: float = 1.0
    hold_ankle_kp: float = 10.0
    contact_force_n: float = 30.0
    landing_timeout_s: float = 1.5
    knee_rad: float = -0.9
    squat_s: float = 1.0
    settle_s: float = 1.0
    # Joint loop: stiff PD toward the planned posture during the squat (M1);
    # afterwards 'sample' reproduces the WBC torque exactly.
    squat_kp: float = 100.0
    squat_ankle_kp: float = 30.0
    squat_kd: float = 2.0
    active_kp: float = 10.0
    active_kd: float = 0.5
    max_tilt_rad: float = 0.5
    activity: str = 'stand'
    sway_amplitude_m: float = 0.03
    sway_period_s: float = 1.5
    sway_ramp_s: float = 1.0
    gains: WBCGains = field(default_factory=WBCGains)


class GaitController:
    def __init__(self, robot: RobotModel, torque_limits, params: GaitParams | None = None):
        self.rm = robot
        self.p = params or GaitParams()
        self.wbc = WholeBodyController(robot, torque_limits, self.p.gains)
        names = robot.joint_names
        self.ankle = np.array(['ankle' in n for n in names])
        self.q_nominal = np.array([self.p.knee_rad if 'knee' in n else
                                   -self.p.knee_rad / 2 if ('hip1' in n or 'ankle' in n) else 0.0
                                   for n in names])
        self.phase, self.fault, self.t_phase = HOLD, '', 0.0
        self.contact_since = None
        self.status: dict = {}
        self.log: list[dict] = []

    # ----------------------------------------------------------- helpers
    def _fail(self, why):
        self.phase, self.fault = FAULT, why
        return None

    def _tilt(self):
        z = self.rm.frame('torso').rotation[:, 2]
        return float(np.arccos(np.clip(z[2], -1, 1)))

    def _hold(self, joints):
        kp = np.where(self.ankle, self.p.hold_ankle_kp, self.p.hold_kp)
        zeros = np.zeros_like(joints)
        return Command(zeros, zeros.copy(), kp, np.full_like(joints, self.p.hold_kd), zeros.copy())

    def _sample_command(self, result, joints, rates):
        dt, p = self.p.dt, self.p
        ddq = result.ddq[6:]
        kp = np.full_like(joints, p.active_kp)
        kd = np.full_like(joints, p.active_kd)
        q_ref = joints + rates * dt + 0.5 * ddq * dt**2
        dq_ref = rates + ddq * dt
        return Command(q_ref, dq_ref, kp, kd, result.tau - kp * (q_ref - joints) - kd * (dq_ref - rates))

    # -------------------------------------------------------------- step
    def step(self, t, q, v, forces, base_held):
        q, v = np.asarray(q, float), np.asarray(v, float)
        joints, rates = q[7:], v[6:]
        if self.phase == FAULT:
            return None
        self.rm.forward(q, v)
        self.forces = np.asarray(forces, float)
        tilt = self._tilt()
        self.status = {'phase': self.phase, 'tilt': tilt}
        if base_held:
            return self._hold(joints)
        if self.phase == HOLD:
            self.phase, self.t_phase = LANDING, t
        if tilt > self.p.max_tilt_rad:
            return self._fail(f'Torso tilt {tilt:.2f} rad')
        if self.phase == LANDING:
            if min(forces) > self.p.contact_force_n:
                self.contact_since = t if self.contact_since is None else self.contact_since
            else:
                self.contact_since = None
            if self.contact_since is not None and t - self.contact_since >= 0.05:
                self._start_squat(t, q)
            elif t - self.t_phase > self.p.landing_timeout_s:
                return self._fail('Feet did not land')
            else:
                return self._hold(joints)
        if self.phase == SQUAT:
            return self._squat(t, q, v, joints, rates)
        return self._active(t, q, v, joints, rates)

    # ------------------------------------------------------------- squat
    def _start_squat(self, t, q):
        self.phase, self.t_phase = SQUAT, t
        for foot in FEET:
            self.wbc.set_mode(foot, FLAT, q)
        # Without ankle roll the lateral CoM is not actuated with both feet flat.
        self.wbc.set_com_mask(self._lateral_mask())
        self.q_land = q[7:].copy()
        self.com_start = self.rm.com.copy()
        target = self.com_start.copy()
        target[:2] = 0.5 * (self.rm.sole_center(FEET[0])[:2] + self.rm.sole_center(FEET[1])[:2])
        target[2] = self._nominal_com_height(q)
        self.com_target = target
        self.ground_z = float(np.mean([self.rm.sole_points(f)[2].mean() for f in FEET]))
        torso = self.rm.frame('torso').rotation
        self.yaw = float(np.arctan2(torso[1, 0], torso[0, 0]))
        self.wbc.set_torso_yaw(self.yaw)
        self.wbc.set_posture(self.q_nominal)

    def _lateral_mask(self):
        axis = np.abs(self.rm.sole_center(FEET[0]) - self.rm.sole_center(FEET[1]))[:2]
        mask = np.ones(3)
        mask[int(np.argmax(axis))] = 0.0
        return mask

    def _nominal_com_height(self, q):
        sole = np.mean([self.rm.sole_points(f)[2].mean() for f in FEET])
        nominal = q.copy()
        nominal[7:] = self.q_nominal
        self.rm.forward(nominal)
        rel = self.rm.com[2] - np.mean([self.rm.sole_points(f)[2].mean() for f in FEET])
        self.rm.forward(q)
        return float(sole + rel)

    def _squat(self, t, q, v, joints, rates):
        p = self.p
        s, ds, dds = min_jerk((t - self.t_phase) / p.squat_s)
        delta = self.com_target - self.com_start
        self.wbc.set_com(self.com_start + s * delta, ds / p.squat_s * delta, dds / p.squat_s**2 * delta)
        try:
            result = self.wbc.solve(q, v, t)
        except RuntimeError as exc:
            return self._fail(str(exc))
        if t - self.t_phase >= p.squat_s + p.settle_s:
            self._start_active(t, q)
        path = self.q_nominal - self.q_land
        kp = np.where(self.ankle, p.squat_ankle_kp, p.squat_kp)
        return Command(self.q_land + s * path, ds / p.squat_s * path, kp,
                       np.full_like(joints, p.squat_kd), result.tau)

    # ------------------------------------------------------------ active
    def _start_active(self, t, q):
        self.phase, self.t_active = ACTIVE, t
        self.z0 = float(self.com_target[2])
        self.omega = math.sqrt(GRAVITY / (self.z0 - self.ground_z))
        self.mid = 0.5 * (self.rm.sole_center(FEET[0]) + self.rm.sole_center(FEET[1]))
        lateral = self.rm.sole_center(FEET[0])[:2] - self.rm.sole_center(FEET[1])[:2]
        self.lat_axis = np.array([*(lateral / np.linalg.norm(lateral)), 0.0])  # towards the left foot
        self.lean = None  # 'left' / 'right': side both feet roll towards

    def _active(self, t, q, v, joints, rates):
        p = self.p
        com, vcom = self.rm.com.copy(), self.rm.vcom.copy()
        if p.activity == 'sway':
            tau_t = t - self.t_active
            ramp = min(1.0, tau_t / p.sway_ramp_s)
            w = 2 * math.pi / p.sway_period_s
            a = p.sway_amplitude_m * ramp
            y, dy, ddy = a * math.sin(w * tau_t), a * w * math.cos(w * tau_t), -a * w * w * math.sin(w * tau_t)
            # Both feet roll towards the side the CoM is on (legs tilt as a
            # parallelogram): leaning left = left foot outer, right foot inner.
            lean = 'left' if y > 0.002 else 'right' if y < -0.002 else self.lean
            if lean != self.lean:
                modes = ({FEET[0]: OUTER, FEET[1]: INNER} if lean == 'left'
                         else {FEET[0]: INNER, FEET[1]: OUTER})
                for foot, mode in modes.items():
                    self.wbc.set_mode(foot, mode, q)
                self.lean = lean
            self.wbc.set_com_mask(np.ones(3) if self.lean else self._lateral_mask())
            ref = self.com_target + y * self.lat_axis
            vel, acc = dy * self.lat_axis, ddy * self.lat_axis
            cop = ref - acc / self.omega**2
            cop[2] = self.ground_z
            self.wbc.set_cop(cop)
        else:
            ref, vel, acc = self.com_target, np.zeros(3), np.zeros(3)
        self.wbc.set_com(ref, vel, acc)
        try:
            result = self.wbc.solve(q, v, t)
        except RuntimeError as exc:
            return self._fail(str(exc))
        lat = float((com - self.mid) @ self.lat_axis)
        lat_ref = float((ref - self.mid) @ self.lat_axis)
        self.status.update({'lat': lat, 'lat_ref': lat_ref, 'edge': getattr(self, 'lean', None),
                            'com_z': float(com[2]), 'forces': result.forces, 'solve_ms': result.solve_ms})
        return self._sample_command(result, joints, rates)
