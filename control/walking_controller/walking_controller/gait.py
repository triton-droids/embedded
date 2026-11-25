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
import pinocchio as pin

from .offline import Command
from .robot import FEET, RobotModel
from .wbc import FLAT, INNER, OUTER, WBCGains, WholeBodyController

HOLD, LANDING, SQUAT, ACTIVE, FAULT = 'hold', 'landing', 'squat', 'active', 'fault'
GRAVITY = 9.81
TORSO_IK_WEIGHT = 0.05


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
    # Latency compensation: the state is predicted (first order, q + v*h) to
    # when the command acts, h = input age + predict_s; predict_s is the fixed
    # delay after the cycle starts (compute, transport, actuation; ~10 ms on
    # the Jetson chain, and the offline loop's default 10 ms command delay).
    # The controller clock is shifted by predict_s too. 0 turns it off.
    predict_s: float = 0.010
    activity: str = 'stand'
    # activity 'squat': both feet flat, the CoM goes down by squat_depth_m (min-jerk over
    # squat_down_s), stays squat_hold_s, comes back up over squat_up_s, then rests squat_rest_s;
    # squat_cycles times (0 = forever). Joint space like the start-up squat: the deep posture comes
    # from inverse kinematics (feet and torso orientation fixed, CoM over the same ground point,
    # squat_depth_m lower), the joints follow the path between the two postures with the squat
    # PD gains, and the WBC torque (CoM on the same path) is the feedforward. The WBC alone could
    # not hold a top-heavy robot (full body, 32 kg) with both feet down: its soft contacts let the
    # QP plan leg accelerations the closed chain cannot follow, and the CoM drifted back until it
    # tipped over.
    squat_depth_m: float = 0.15
    squat_down_s: float = 2.0
    squat_hold_s: float = 1.0
    squat_up_s: float = 2.0
    squat_rest_s: float = 1.0
    squat_cycles: int = 0
    # Joint PD during the squat activity. The ankles need a stiffer loop than the start-up squat's
    # (30): at 30 the full-body robot tipped back on its first squat, at 100 it squats down 8 cm
    # (offline, the ankles then reach their 17 Nm).
    # Where the deep posture puts the CoM (horizontally): 'start' keeps it where the squat began; 'ankle' puts
    # it squat_com_ahead_m in front of the midpoint of the ankle axes; 'sole' over the middle of the soles.
    # The full-body robot's ankles sit near the heel and give 17 Nm: with 'start' the CoM began 5 cm ahead and
    # ran to the toes as the knees went forward, the ankles saturated and it tipped forward.
    squat_com_ref: str = 'ankle'
    squat_com_ahead_m: float = 0.02
    squat_activity_ankle_kp: float = 100.0
    squat_activity_kd: float = 4.0
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
        # Hip pitch less by the torso's back lean (WBCGains.torso_pitch_rad), so the start-up squat already
        # ends in the posture the WBC then holds.
        lean = self.p.gains.torso_pitch_rad
        self.q_nominal = np.array([self.p.knee_rad if 'knee' in n else
                                   -self.p.knee_rad / 2 - lean if 'hip1' in n else
                                   -self.p.knee_rad / 2 if 'ankle' in n else 0.0
                                   for n in names])
        self.phase, self.fault, self.t_phase = HOLD, '', 0.0
        self.contact_since = None
        self.status: dict = {}
        self.log: list[dict] = []
        self.horizon_s = 0.0      # last prediction horizon (status)

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
    def predict(self, q, v, horizon):
        """State `horizon` seconds ahead at constant velocity. (Extrapolating
        with the last WBC acceleration was worse: it falls at 30 ms delay.)"""
        return pin.integrate(self.rm.model, q, v * horizon), v

    def step(self, t, q, v, forces, base_held, age_s=0.0):
        """age_s: how old (q, v) is at time t (sample stamp to now)."""
        q, v = np.asarray(q, float), np.asarray(v, float)
        horizon = max(0.0, float(age_s)) + self.p.predict_s
        self.horizon_s = 0.0
        if horizon > 0.0 and not base_held and self.phase not in (HOLD, FAULT):
            q, v = self.predict(q, v, horizon)
            t = t + self.p.predict_s
            self.horizon_s = horizon
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

    def start_from_stand(self, t, q, v, forces):
        """Take over a robot already standing on both feet (after a fall recovery): skip LANDING, which
        holds every joint at zero until the feet load (meant for lowering from the gantry, and would
        jerk a crouched robot straight), and squat from the current joints to the start stance."""
        q, v = np.asarray(q, float), np.asarray(v, float)
        self.rm.forward(q, v)
        self.forces = np.asarray(forces, float)
        self.fault = ''
        self._start_squat(t, q)

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

    def _squat_activity(self, t, q, v, joints, rates):
        p = self.p
        if not hasattr(self, 'squat_ik'):
            self.q_top = joints.copy()
            self.torso_top = self.rm.frame('torso').rotation.copy()
            self.com_top = self.rm.com.copy()
            # Stand back up into q_up (same height, CoM at the goal), not into q_top: the start-up stance leaves
            # the full-body robot's CoM ahead of its ankles, and standing back into it tipped it forward.
            self.com_up = self._squat_com_goal(0.0)
            self.com_deep = self._squat_com_goal(p.squat_depth_m)
            # The two IK solves take 15-40 ms, over the 30 ms cycle budget on the ROS chain: a few iterations per
            # cycle, holding the start posture meanwhile (about 12 cycles).
            self.squat_ik = [self._squat_posture_steps(q.copy(), 0.0, self.com_up),
                             self._squat_posture_steps(q.copy(), p.squat_depth_m, self.com_deep)]
            self.squat_ik_out = []
        if len(self.squat_ik_out) < 2:
            try:
                next(self.squat_ik[len(self.squat_ik_out)])
            except StopIteration as done:
                self.squat_ik_out.append(done.value)
            if len(self.squat_ik_out) < 2:
                return self._squat_hold(t, q, v, joints)
            (self.q_up, self.torso_up), (self.q_deep, self.torso_deep) = self.squat_ik_out
            self.torso_lean = pin.log3(self.torso_top.T @ self.torso_deep)
            self.torso_lean_up = pin.log3(self.torso_top.T @ self.torso_up)
            self.t_squat = t
            self.rm.forward(q, v)
        tau = t - self.t_squat
        dz, dvz, daz = self._squat_offset(tau)
        s, ds = -dz / p.squat_depth_m, -dvz / p.squat_depth_m
        # Going down (and holding) from the start posture, coming up into q_up: the path's upper end changes
        # once the first descent is over.
        rising = tau >= p.squat_down_s + p.squat_hold_s
        q0, c0, lean0 = (self.q_up, self.com_up, self.torso_lean_up) if rising else (self.q_top, self.com_top, np.zeros(3))
        path = self.q_deep - q0
        shift = self.com_deep - c0
        shift[2] = 0.0
        top = c0.copy(); top[2] = self.com_top[2]
        self.wbc.set_com(top + s * shift + np.array([0.0, 0.0, dz]),
                         ds * shift + np.array([0.0, 0.0, dvz]), np.array([0.0, 0.0, daz]))
        self.wbc.set_posture(q0 + s * path)
        self.wbc.set_torso_rotation(self.torso_top @ pin.exp3(lean0 + s * (self.torso_lean - lean0)))
        try:
            result = self.wbc.solve(q, v, t)
        except RuntimeError as exc:
            return self._fail(str(exc))
        self.status.update({'squat_dz': dz, 'com_z': float(self.rm.com[2]), 'forces': result.forces})
        kp = np.where(self.ankle, p.squat_activity_ankle_kp, p.squat_kp)
        return Command(q0 + s * path, ds * path, kp, np.full_like(joints, p.squat_activity_kd),
                       result.tau)

    def _squat_com_goal(self, depth):
        p = self.p
        goal = self.rm.com.copy()
        goal[2] -= depth
        if p.squat_com_ref == 'start':
            return goal
        if p.squat_com_ref == 'sole':
            ref = 0.5 * (self.rm.sole_center(FEET[0]) + self.rm.sole_center(FEET[1]))
        else:  # 'ankle': ankle axes (foot frame origins), a little forward
            ref = 0.5 * (self.rm.frame(FEET[0]).translation + self.rm.frame(FEET[1]).translation)
            torso_y = self.rm.frame('torso').rotation[:, 1]
            fwd = np.array([torso_y[0], torso_y[1], 0.0]); fwd /= np.linalg.norm(fwd)
            ref = ref + p.squat_com_ahead_m * fwd
        goal[:2] = ref[:2]
        return goal

    def _squat_hold(self, t, q, v, joints):
        """Start posture while the squat IK runs."""
        p = self.p
        self.wbc.set_com(self.com_top, np.zeros(3), np.zeros(3))
        self.wbc.set_posture(self.q_top)
        try:
            result = self.wbc.solve(q, v, t)
        except RuntimeError as exc:
            return self._fail(str(exc))
        kp = np.where(self.ankle, p.squat_activity_ankle_kp, p.squat_kp)
        return Command(self.q_top.copy(), np.zeros_like(joints), kp, np.full_like(joints, p.squat_activity_kd),
                       result.tau)

    def _squat_posture(self, q, depth, com_goal=None, iterations=30):
        """IK posture (see _squat_posture_steps), solved in one call."""
        gen = self._squat_posture_steps(q, depth, com_goal, iterations, per_step=iterations)
        while True:
            try:
                next(gen)
            except StopIteration as done:
                qj, self.torso_deep = done.value
                return qj

    def _squat_posture_steps(self, q, depth, com_goal=None, iterations=30, per_step=5):
        """Generator: yields every `per_step` iterations, returns (joint posture, torso rotation)."""
        q = q.copy()
        yield from ()
        result = yield from self._squat_posture_gen(q, depth, com_goal, iterations, per_step)
        return result

    def _squat_posture_gen(self, q, depth, com_goal=None, iterations=200, per_step=10**9):
        """Joint posture with both feet and the torso orientation where they are now and the CoM
        `depth` lower over the same ground point (damped least squares on Pinocchio)."""
        model, data = self.rm.model, self.rm.model.createData()
        pin.framesForwardKinematics(model, data, q)
        feet = {f: data.oMf[self.rm.frame_id[f]].copy() for f in FEET}
        torso = data.oMf[self.rm.frame_id['torso']].rotation.copy()
        com = pin.centerOfMass(model, data, q).copy()
        com[2] -= depth
        if com_goal is not None:
            com = np.asarray(com_goal, float).copy()
        q = q.copy()
        prev = np.inf
        for it in range(iterations):
            if it and it % per_step == 0:
                yield
            pin.computeJointJacobians(model, data, q)
            pin.framesForwardKinematics(model, data, q)
            rows, errs = [], []
            for f, target in feet.items():
                fid = self.rm.frame_id[f]
                errs.append(pin.log6(data.oMf[fid].actInv(target)).vector)
                rows.append(pin.getFrameJacobian(model, data, fid, pin.LOCAL))
            fid = self.rm.frame_id['torso']
            # Low weight: with the torso upright the ankles (+-34 deg) cannot keep the CoM over the
            # feet in a deep squat, so the torso leans forward as much as needed.
            errs.append(TORSO_IK_WEIGHT * pin.log3(data.oMf[fid].rotation.T @ torso))
            rows.append(TORSO_IK_WEIGHT * pin.getFrameJacobian(model, data, fid, pin.LOCAL)[3:])
            errs.append(com - pin.centerOfMass(model, data, q))
            rows.append(pin.jacobianCenterOfMass(model, data, q))
            J, e = np.vstack(rows), np.concatenate(errs)
            norm = float(np.linalg.norm(e))
            if norm < 1e-5 or prev - norm < 1e-6:  # converged, or no longer improving (limits)
                break
            prev = norm
            # Minimum-norm step (the arms and redundant joints move as little as possible). A joint at
            # its limit that the step would push further is locked (its column dropped), so the
            # others take over; plain clipping stalled at the ankle's +34 deg.
            free = np.ones(model.nv, bool)
            for _ in range(model.nv):
                Jf = J * free
                dq = Jf.T @ np.linalg.solve(Jf @ Jf.T + 1e-6 * np.eye(len(e)), e)
                joint_q = q[7:] + 0.5 * dq[6:]
                over = (((joint_q > model.upperPositionLimit[7:]) & (dq[6:] > 0)) |
                        ((joint_q < model.lowerPositionLimit[7:]) & (dq[6:] < 0)))
                if not (over & free[6:]).any():
                    break
                free[6:][over] = False
            q = pin.integrate(model, q, 0.5 * dq)
            q[7:] = np.clip(q[7:], model.lowerPositionLimit[7:], model.upperPositionLimit[7:])
        e = np.concatenate(errs[:2] + errs[3:])  # feet and CoM; the torso may lean
        self.status['squat_ik_error'] = float(np.linalg.norm(e))
        pin.framesForwardKinematics(model, data, q)
        return q[7:].copy(), data.oMf[self.rm.frame_id['torso']].rotation.copy()

    def _squat_offset(self, tau):
        """CoM height offset (m), its rate and acceleration, `tau` s into the squat activity."""
        p = self.p
        period = p.squat_down_s + p.squat_hold_s + p.squat_up_s + p.squat_rest_s
        if p.squat_cycles and tau >= p.squat_cycles * period:
            return 0.0, 0.0, 0.0
        tau = tau % period
        d = p.squat_depth_m
        if tau < p.squat_down_s:
            s, ds, dds = min_jerk(tau / p.squat_down_s)
            return -d * s, -d * ds / p.squat_down_s, -d * dds / p.squat_down_s**2
        tau -= p.squat_down_s
        if tau < p.squat_hold_s:
            return -d, 0.0, 0.0
        tau -= p.squat_hold_s
        if tau < p.squat_up_s:
            s, ds, dds = min_jerk(tau / p.squat_up_s)
            return -d * (1 - s), d * ds / p.squat_up_s, d * dds / p.squat_up_s**2
        return 0.0, 0.0, 0.0

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
        elif p.activity == 'squat':
            return self._squat_activity(t, q, v, joints, rates)
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
