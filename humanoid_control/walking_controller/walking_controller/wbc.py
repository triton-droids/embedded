"""TSID whole-body controller with per-foot contact modes.

Each foot is in one of three modes:

- 'flat':  6D surface contact (Contact6d on the four sole corners)
- 'inner' / 'outer': line contact on that edge (two ContactPoint), so the
           foot can roll about it. Without ankle roll the CoM can move sideways
           in double support only if BOTH feet roll on parallel edges (the legs
           tilt like a parallelogram): CoM to the right of the feet needs the
           left foot on its inner and the right foot on its outer edge. With
           one or both feet flat the lateral CoM authority is ~5e-4 (locked).
- 'swing': no contact; SE3 position task, orientation task with lower weight

Tasks: CoM (acceleration feedforward from the LIPM/MPC), CoP (MPC reference),
centroidal angular momentum (held at zero), pelvis orientation, joint posture.
Lessons from the earlier stand experiment are kept: soft contacts with a large
weight and f_min = 0, masks set before a task is added, CoP-bound CoM gains.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np
import pinocchio as pin
import tsid

from .robot import FEET, TORSO, RobotModel

FLAT, INNER, OUTER, SWING = 'flat', 'inner', 'outer', 'swing'
EDGES = (INNER, OUTER)


def _kd(kp):
    return 2.0 * np.sqrt(kp)


@dataclass
class WBCGains:
    com_kp: float = 15.0
    com_kp_z: float = 40.0
    com_weight: float = 1.0
    cop_weight: float = 1e-3
    am_kp: float = 10.0
    am_weight: float = 0.05
    torso_kp: float = 50.0
    torso_weight: float = 0.5
    posture_kp: float = 20.0
    posture_weight: float = 0.01
    contact_kp: float = 100.0
    contact_weight: float = 1e3
    force_regularization_weight: float = 1e-5
    mu: float = 0.6
    f_max: float = 400.0
    swing_kp: float = 200.0
    swing_weight: float = 3.0
    swing_orientation_weight: float = 0.3


@dataclass
class WBCResult:
    tau: np.ndarray
    ddq: np.ndarray
    forces: dict = field(default_factory=dict)
    solve_ms: float = 0.0


class WholeBodyController:
    def __init__(self, robot: RobotModel, torque_limits, gains: WBCGains | None = None):
        self.rm = robot
        self.g = g = gains or WBCGains()
        self.robot = tsid.RobotWrapper(robot.model, tsid.RootJointType.FLOATING_BASE_SYSTEM, False)
        self.robot.set_rotor_inertias(robot.armature)
        self.robot.set_gear_ratios(np.ones(robot.na))
        self.data = self.robot.data()
        self.form = tsid.InverseDynamicsFormulationAccForce('walk_wbc', self.robot, False)
        self.solver = tsid.SolverHQuadProgFast('qp')
        self.limits = np.asarray(torque_limits, float)

        self.com = tsid.TaskComEquality('com', self.robot)
        kp = np.array([g.com_kp, g.com_kp, g.com_kp_z])
        self.com.setKp(kp)
        self.com.setKd(_kd(kp))
        self.com_mask = np.ones(3)
        self.form.addMotionTask(self.com, g.com_weight, 1, 0.0)

        self.am = tsid.TaskAMEquality('am', self.robot)
        self.am.setKp(np.full(3, g.am_kp))
        self.am.setKd(np.full(3, _kd(g.am_kp)))
        self.form.addMotionTask(self.am, g.am_weight, 1, 0.0)
        zero = tsid.TrajectorySample(3)
        zero.value(np.zeros(3)); zero.derivative(np.zeros(3)); zero.second_derivative(np.zeros(3))
        self.am.setReference(zero)

        self.torso = tsid.TaskSE3Equality('torso', self.robot, TORSO)
        self.torso.setKp(np.full(6, g.torso_kp))
        self.torso.setKd(np.full(6, _kd(g.torso_kp)))
        self.torso.setMask(np.array([0., 0., 0., 1., 1., 1.]))
        self.form.addMotionTask(self.torso, g.torso_weight, 1, 0.0)

        self.posture = tsid.TaskJointPosture('posture', self.robot)
        self.posture.setKp(np.full(robot.na, g.posture_kp))
        self.posture.setKd(np.full(robot.na, _kd(g.posture_kp)))
        self.form.addMotionTask(self.posture, g.posture_weight, 1, 0.0)

        self.cop = tsid.TaskCopEquality('cop', self.robot)
        self.cop_active = False

        self.bounds = tsid.TaskActuationBounds('torque_bounds', self.robot)
        self.form.addActuationTask(self.bounds, 1.0, 0, 0.0)

        self.modes = {f: None for f in FEET}
        self.contacts: dict[str, list[str]] = {f: [] for f in FEET}
        self.contact_objects: dict[str, object] = {}
        self.swing: dict[str, tuple] = {}
        self._resize()

    def _resize(self):
        self.solver.resize(self.form.nVar, self.form.nEq, self.form.nIn)

    # ------------------------------------------------------------- modes
    def set_mode(self, foot: str, mode: str, q):
        if self.modes[foot] == mode:
            return
        self._clear(foot)
        pin.framesForwardKinematics(self.rm.model, self.rm.data, q)
        if mode == FLAT:
            self._add_flat(foot)
        elif mode in EDGES:
            self._add_edge(foot, mode)
        elif mode == SWING:
            self._add_swing(foot)
        else:
            raise ValueError(mode)
        self.modes[foot] = mode
        self._resize()

    def _clear(self, foot):
        for name in self.contacts[foot]:
            if not self.form.removeRigidContact(name, 0.0):
                raise RuntimeError(f'Failed to remove {name}')
            self.contact_objects.pop(name)
        self.contacts[foot] = []
        if foot in self.swing:
            for task in self.swing.pop(foot):
                self.form.removeTask(task.name, 0.0)

    def _add_flat(self, foot):
        g = self.g
        pose = self.rm.data.oMf[self.rm.frame_id[foot]].copy()
        normal = pose.rotation.T @ np.array([0., 0., 1.])
        c = tsid.Contact6d(foot + '_flat', self.robot, foot, self.rm.contact_points[foot],
                           normal, g.mu, 0.0, g.f_max)
        c.setKp(np.full(6, g.contact_kp))
        c.setKd(np.full(6, _kd(g.contact_kp)))
        c.setReference(pose)
        self.form.addRigidContact(c, g.force_regularization_weight, g.contact_weight, 1)
        self.contacts[foot] = [c.name]
        self.contact_objects[c.name] = c

    def _add_edge(self, foot, side):
        g = self.g
        names = []
        inner = self.rm.inner_corners(foot)
        corners = inner if side == INNER else [i for i in range(4) if i not in inner]
        for index in corners:
            frame = self.rm.corner_frames[foot][index]
            pose = self.rm.data.oMf[self.rm.model.getFrameId(frame)].copy()
            c = tsid.ContactPoint(f'{frame}_point', self.robot, frame, np.array([0., 0., 1.]),
                                  g.mu, 0.0, g.f_max / 2)
            c.useLocalFrame(False)
            c.setKp(np.full(3, g.contact_kp))
            c.setKd(np.full(3, _kd(g.contact_kp)))
            c.setReference(pose)
            self.form.addRigidContact(c, g.force_regularization_weight, g.contact_weight, 1)
            names.append(c.name)
            self.contact_objects[c.name] = c
        self.contacts[foot] = names

    def _add_swing(self, foot):
        g = self.g
        pos = tsid.TaskSE3Equality(foot + '_swing_pos', self.robot, foot)
        pos.setMask(np.array([1., 1., 1., 0., 0., 0.]))
        rot = tsid.TaskSE3Equality(foot + '_swing_rot', self.robot, foot)
        rot.setMask(np.array([0., 0., 0., 1., 1., 1.]))
        for task in (pos, rot):
            task.setKp(np.full(6, g.swing_kp))
            task.setKd(np.full(6, _kd(g.swing_kp)))
        self.form.addMotionTask(pos, g.swing_weight, 1, 0.0)
        self.form.addMotionTask(rot, g.swing_orientation_weight, 1, 0.0)
        self.swing[foot] = (pos, rot)
        self.set_swing_reference(foot, self.rm.data.oMf[self.rm.frame_id[foot]].copy())

    def set_max_force(self, foot, f_max):
        """Ramp a foot's load down before lift-off (per contact object)."""
        names = self.contacts[foot]
        for name in names:
            # A flat foot is one Contact6d; an edge splits its load over two points.
            self.contact_objects[name].setMaxNormalForce(float(f_max) / len(names))

    # -------------------------------------------------------- references
    def set_com(self, position, velocity, acceleration):
        s = tsid.TrajectorySample(3)
        s.value(np.asarray(position, float))
        s.derivative(np.asarray(velocity, float))
        s.second_derivative(np.asarray(acceleration, float))
        self.com.setReference(s)

    def set_com_mask(self, mask):
        mask = np.asarray(mask, float)
        if np.array_equal(mask, self.com_mask):
            return
        # TSID sizes a task when it is added: re-add after changing the mask.
        self.form.removeTask('com', 0.0)
        self.com.setMask(mask)
        self.form.addMotionTask(self.com, self.g.com_weight, 1, 0.0)
        self.com_mask = mask
        self._resize()

    def set_cop(self, cop_world):
        """Track a CoP (world, on the ground plane); None disables the task."""
        if cop_world is None:
            if self.cop_active:
                self.form.removeTask('cop', 0.0)
                self.cop_active = False
                self._resize()
            return
        self.cop.setReference(np.asarray(cop_world, float))
        self.cop.setContactNormal(np.array([0., 0., 1.]))
        if not self.cop_active:
            self.form.addForceTask(self.cop, self.g.cop_weight, 1, 0.0)
            self.cop_active = True
            self._resize()

    def set_torso_yaw(self, yaw):
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        self.torso.setReference(tsid.TrajectorySE3Constant('torso_ref', pin.SE3(R, np.zeros(3))).computeNext())

    def set_posture(self, joints):
        self.posture.setReference(
            tsid.TrajectoryEuclidianConstant('posture_ref', np.asarray(joints, float)).computeNext())

    def set_swing_reference(self, foot, pose: pin.SE3, velocity=None, acceleration=None):
        s = tsid.TrajectorySample(12, 6)
        s.value(np.concatenate([pose.translation, pose.rotation.reshape(-1, order='F')]))
        s.derivative(np.zeros(6) if velocity is None else np.asarray(velocity, float))
        s.second_derivative(np.zeros(6) if acceleration is None else np.asarray(acceleration, float))
        for task in self.swing[foot]:
            task.setReference(s)

    # ------------------------------------------------------------- solve
    def solve(self, q, v, t=0.0) -> WBCResult:
        damping = self.rm.damping * v[6:]
        # MuJoCo applies joint damping; TSID does not model it.
        self.bounds.setBounds(-self.limits - damping, self.limits - damping)
        problem = self.form.computeProblemData(float(t), q, v)
        start = time.monotonic()
        sol = self.solver.solve(problem)
        solve_ms = (time.monotonic() - start) * 1e3
        if int(sol.status) != 0:
            raise RuntimeError(f'QP failed with status {sol.status}')
        tau = np.array(self.form.getActuatorForces(sol)) + damping
        ddq = np.array(self.form.getAccelerations(sol))
        if not (np.isfinite(tau).all() and np.isfinite(ddq).all()):
            raise RuntimeError('QP returned non-finite values')
        if float(np.max(np.abs(tau) - self.limits)) > 1e-3:
            raise RuntimeError('QP violated torque bounds')
        forces = {}
        for foot in FEET:
            total = 0.0
            for name in self.contacts[foot]:
                c = self.contact_objects[name]
                total += float(c.getNormalForce(self.form.getContactForce(name, sol)))
            forces[foot] = total
        return WBCResult(np.clip(tau, -self.limits, self.limits), ddq, forces, solve_ms)
