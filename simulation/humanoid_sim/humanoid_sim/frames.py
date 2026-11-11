"""Planar robot frames shared by the camera and odometry TF publishers. No ROS.

ROS ``base_link`` axes: +x forward, +y left, +z up. The MuJoCo base body uses
different axes (forward along body -Y or +Y depending on the scene), so
``world_from_base_link = world_from_body @ body_from_base(axis)``.

``base_footprint`` (REP-120): on the floor (z = 0) under ``base_link``, yaw of
``base_link`` only (gravity aligned). ``odom`` (REP-105): world fixed,
continuous, origin = the footprint pose at the time it was anchored.
"""

from __future__ import annotations

import math

import numpy as np

FORWARD_AXES = ('negative_y', 'positive_y')


def body_from_base(forward_axis: str) -> np.ndarray:
    """Rotation from ROS base_link axes to MuJoCo base body axes."""
    if forward_axis not in FORWARD_AXES:
        raise ValueError('base_forward_axis must be negative_y or positive_y')
    sign = 1.0 if forward_axis == 'positive_y' else -1.0
    return np.array([[0., -sign, 0.], [sign, 0., 0.], [0., 0., 1.]])


def planar(x: float, y: float, yaw: float) -> np.ndarray:
    """4x4 transform of a pose on the floor."""
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def rigid(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T


def inverse(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    return rigid(R.T, -R.T @ p)


def heading(rotation: np.ndarray) -> float:
    """Yaw of the forward (+x) axis projected on the floor."""
    return math.atan2(rotation[1, 0], rotation[0, 0])


def footprint(world_from_base: np.ndarray) -> np.ndarray:
    """world_from_base_footprint: base_link's floor point and yaw, z = 0."""
    return planar(world_from_base[0, 3], world_from_base[1, 3], heading(world_from_base[:3, :3]))


def yaw_of(T: np.ndarray) -> float:
    return heading(T[:3, :3])


def quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Unit quaternion (x, y, z, w) of a rotation matrix, w >= 0."""
    R = np.asarray(rotation, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
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


def split_odometry(world_from_odom: np.ndarray, world_from_base: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray]:
    """(odom_from_footprint, footprint_from_base) for one base_link pose."""
    world_from_fp = footprint(world_from_base)
    return inverse(world_from_odom) @ world_from_fp, inverse(world_from_fp) @ world_from_base


def footprint_twist(world_from_base: np.ndarray, linear_world: np.ndarray,
                    angular_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Base velocity expressed in base_footprint axes (REP-105 child-frame twist)."""
    R = footprint(world_from_base)[:3, :3]
    return R.T @ np.asarray(linear_world, float), R.T @ np.asarray(angular_world, float)
