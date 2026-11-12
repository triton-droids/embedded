"""Loosely coupled base pose: body-IMU attitude, visual-odometry translation. No ROS.

Inputs
  * Body IMU orientation (BNO085 game rotation vector: gravity-referenced roll and
    pitch, gyro-integrated yaw relative to power-on, no magnetometer).
  * Stereo VO pose of the camera rig in the VO's own odometry frame, plus the
    base->rig transform at the same stamp (head forward kinematics; the camera
    moves relative to the base when the head turns).

Estimate (frame ``world``, child = the IMU-aligned base body)
  * orientation = yaw_offset * IMU orientation. yaw_offset aligns the IMU's
    relative yaw with the known start heading once, at the first VO sample.
  * position: integrated base displacements measured by VO, each rotated into
    ``world`` with the IMU orientation at the start of that displacement.

Using VO only as relative motion keeps the estimate independent of the VO's
arbitrary odometry frame and lets single implausible VO steps be rejected
without discarding the accumulated position.

All transforms are 4x4 homogeneous matrices; quaternions are (w, x, y, z).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def quat_to_matrix(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(n) or n < 1e-6:
        raise ValueError('Invalid quaternion')
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """(w, x, y, z) with w >= 0."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    return q if q[0] >= 0 else -q


def transform(rotation: np.ndarray, translation) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def invert(t: np.ndarray) -> np.ndarray:
    r = t[:3, :3]
    return transform(r.T, -r.T @ t[:3, 3])


def yaw_of(r: np.ndarray) -> float:
    """Heading of the body x axis projected on the ground plane."""
    return math.atan2(r[1, 0], r[0, 0])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


@dataclass
class FusionConfig:
    initial_position: tuple = (0.0, 0.0, 0.0)  # Base position in world at the first VO sample.
    initial_yaw: float = 0.0                    # Base heading in world at the first VO sample.
    max_speed_m_s: float = 2.0                  # Reject VO steps implying a faster base.
    max_step_gap_s: float = 0.5                 # Longer VO gaps restart from the new sample.
    imu_history_s: float = 2.0


class VioFusion:
    def __init__(self, config: FusionConfig | None = None):
        self.config = config or FusionConfig()
        self._imu = deque()  # (stamp_s, world-uncorrected rotation of the base)
        self._yaw_offset = None
        self.position = None
        self._previous = None  # (stamp_s, T_odom_rig, T_base_rig)
        self.accepted = 0
        self.rejected = 0
        self.restarts = 0

    @property
    def initialized(self) -> bool:
        return self.position is not None

    def add_imu(self, stamp_s: float, quat_wxyz, base_from_imu: np.ndarray | None = None) -> None:
        r = quat_to_matrix(quat_wxyz)
        if base_from_imu is not None:
            r = r @ base_from_imu.T
        if self._imu and stamp_s < self._imu[-1][0]:
            return  # Out of order.
        self._imu.append((stamp_s, r))
        while self._imu and self._imu[0][0] < stamp_s - self.config.imu_history_s:
            self._imu.popleft()

    def _imu_at(self, stamp_s: float) -> np.ndarray | None:
        """Nearest IMU attitude to stamp_s (100 Hz IMU: at most 5 ms away)."""
        if not self._imu:
            return None
        best = min(self._imu, key=lambda item: abs(item[0] - stamp_s))
        return best[1] if abs(best[0] - stamp_s) < 0.05 else None

    def orientation(self, stamp_s: float | None = None) -> np.ndarray | None:
        if self._yaw_offset is None or not self._imu:
            return None
        r = self._imu[-1][1] if stamp_s is None else self._imu_at(stamp_s)
        return None if r is None else rot_z(self._yaw_offset) @ r

    def add_vo(self, stamp_s: float, odom_from_rig: np.ndarray, base_from_rig: np.ndarray) -> str:
        """One VO pose with the base->rig transform at its stamp. Returns the outcome."""
        attitude = self._imu_at(stamp_s)
        if attitude is None:
            return 'no_imu'
        if self._yaw_offset is None:
            self._yaw_offset = self.config.initial_yaw - yaw_of(attitude)
            self.position = np.asarray(self.config.initial_position, dtype=float)
            self._previous = (stamp_s, odom_from_rig, base_from_rig)
            return 'initialized'
        previous_stamp, previous_odom_rig, previous_base_rig = self._previous
        dt = stamp_s - previous_stamp
        if dt <= 0.0:
            return 'stale'
        self._previous = (stamp_s, odom_from_rig, base_from_rig)
        if dt > self.config.max_step_gap_s:
            self.restarts += 1
            return 'restarted'
        # Base motion expressed in the previous base frame.
        rig_motion = invert(previous_odom_rig) @ odom_from_rig
        base_motion = previous_base_rig @ rig_motion @ invert(base_from_rig)
        step = base_motion[:3, 3]
        if not np.isfinite(step).all() or np.linalg.norm(step) > self.config.max_speed_m_s * dt:
            self.rejected += 1
            return 'rejected'
        world_from_previous_base = self.orientation(previous_stamp)
        if world_from_previous_base is None:
            self.rejected += 1
            return 'no_imu'
        self.position = self.position + world_from_previous_base @ step
        self.accepted += 1
        return 'accepted'
