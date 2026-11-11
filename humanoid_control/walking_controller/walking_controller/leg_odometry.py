"""Short-horizon leg odometry: IMU attitude + stance-foot kinematics. No ROS.

What a robot without a state estimator can still know:

* attitude from the body IMU (BNO085 Game Rotation Vector: roll/pitch from
  gravity, yaw integrated from the gyro relative to power-on, slow drift);
* where the feet are relative to the base, from joint angles (forward
  kinematics of the leg model).

A foot that carries the robot does not move on the floor. So while one foot is
the stance foot its floor position is an anchor, and the base is at
``anchor - foot(base)`` (``foot(base)``: the foot relative to the base, rotated
into the world by the IMU attitude). When the other foot becomes the stance
foot, its current world position becomes the new anchor. There are no foot
contact sensors, and this gait lifts the swing foot only ~1 cm, so the stance
foot comes from the walking controller (``/walk/stance``, the foot it puts in
single support); without it, the lower foot (with a hysteresis margin).
The floor is assumed flat (anchors at z = 0).

Errors grow with distance walked (foot slip, kinematic and joint offsets) and
with IMU yaw drift; they are meant to be small over the seconds a ball tracker
needs, not over a match. ``odom`` is anchored at the first footprint pose
(REP-105 ``odom``), like the simulator's ground-truth ``sim_odom_tf``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

FEET = ('left_foot', 'right_foot')  # as robot.FEET; no Pinocchio import here (tests, tools)

FORWARD_AXES = ('negative_y', 'positive_y')


def body_from_base(forward_axis: str) -> np.ndarray:
    """Rotation from ROS base_link axes (x forward) to the model's base body axes."""
    if forward_axis not in FORWARD_AXES:
        raise ValueError('base_forward_axis must be negative_y or positive_y')
    sign = 1.0 if forward_axis == 'positive_y' else -1.0
    return np.array([[0., -sign, 0.], [sign, 0., 0.], [0., 0., 1.]])


def quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not n > 0.0 or not math.isfinite(n):
        raise ValueError('invalid quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_xyzw(R) -> tuple[float, float, float, float]:
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return tuple(float(v) for v in q)


def rigid(R, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def inverse(T) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    return rigid(R.T, -R.T @ p)


def footprint(T) -> np.ndarray:
    """Floor point under T and its heading (yaw of T's x axis), z = 0."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    F = np.eye(4)
    F[:2, :2] = [[c, -s], [s, c]]
    F[0, 3], F[1, 3] = T[0, 3], T[1, 3]
    return F


@dataclass
class OdometryEstimate:
    odom_from_base: np.ndarray       # 4x4, base_link in odom
    odom_from_footprint: np.ndarray  # 4x4, base_footprint in odom (z = 0, yaw only)
    footprint_from_base: np.ndarray  # 4x4
    stance: str
    switches: int


class LegOdometry:
    """Integrates IMU attitude and stance-foot kinematics into an odom-frame base pose.

    ``robot``: walking_controller.robot.RobotModel (state_from_sim, forward, sole_center, na).
    """

    def __init__(self, robot, forward_axis: str = 'positive_y',
                 imu_to_body: Optional[np.ndarray] = None, switch_margin_m: float = 0.01):
        self.robot = robot
        self.body_from_base = body_from_base(forward_axis)
        # body <- sensor rotation; identity when the IMU is aligned with the base body.
        self.imu_to_body = np.eye(3) if imu_to_body is None else np.asarray(imu_to_body, float)
        self.margin = float(switch_margin_m)
        self.reset()

    def reset(self) -> None:
        self.stance: Optional[str] = None
        self.anchor = np.zeros(3)
        self.world_from_odom: Optional[np.ndarray] = None
        self.switches = 0

    def feet(self, joint_positions, world_from_body: np.ndarray) -> dict[str, np.ndarray]:
        """Sole centers relative to the base origin, in world axes."""
        quat = matrix_to_quat_xyzw(world_from_body)
        zeros = np.zeros(3)
        q, _ = self.robot.state_from_sim(zeros, quat, zeros, zeros,
                                         np.asarray(joint_positions, float),
                                         np.zeros(self.robot.na))
        self.robot.forward(q)
        return {f: self.robot.sole_center(f).copy() for f in FEET}

    def update(self, joint_positions, imu_quat_xyzw, stance: Optional[str] = None) -> OdometryEstimate:
        """``stance``: the support foot if known (the walking controller's single-support foot);
        None falls back to the lower foot with a hysteresis margin."""
        world_from_body = quat_xyzw_to_matrix(imu_quat_xyzw) @ self.imu_to_body.T
        feet = self.feet(joint_positions, world_from_body)
        if stance is not None and stance not in FEET:
            raise ValueError(f'unknown stance foot {stance!r}')
        if self.stance is None:
            self.stance = stance or min(FEET, key=lambda f: feet[f][2])
            self.anchor = np.array([feet[self.stance][0], feet[self.stance][1], 0.0])
        else:
            other = FEET[1] if self.stance == FEET[0] else FEET[0]
            switch = (stance == other) if stance is not None else (
                feet[other][2] < feet[self.stance][2] - self.margin)
            if switch:
                base = self.anchor - feet[self.stance]
                landed = base + feet[other]
                self.anchor = np.array([landed[0], landed[1], 0.0])  # flat floor
                self.stance = other
                self.switches += 1
        base = self.anchor - feet[self.stance]
        world_from_base = rigid(world_from_body @ self.body_from_base, base)
        if self.world_from_odom is None:
            self.world_from_odom = footprint(world_from_base)
        odom_from_base = inverse(self.world_from_odom) @ world_from_base
        odom_from_fp = footprint(odom_from_base)
        return OdometryEstimate(odom_from_base, odom_from_fp, inverse(odom_from_fp) @ odom_from_base,
                                self.stance, self.switches)
