"""Simulated micro-ROS MCUs: the body BNO085 IMU and the head IMU/servo board.

Body IMU (replaces policy_imu_microros firmware + micro_ros_agent):
  /imu/data_raw  sensor_msgs/Imu, frame imu_link, 100 Hz
                 orientation    sensor-to-world quaternion (like Game RV; yaw is
                                relative to the spawn heading, not a compass)
                 angular_vel    rad/s in the sensor frame
                 linear_accel   specific force in m/s^2, ~[0, 0, +9.81] upright
  /imu/status    std_msgs/String JSON, 1 Hz

Head MCU (replaces humanoid_vision_microros firmware + micro_ros_agent):
  /head/imu               sensor_msgs/Imu without orientation (covariance[0] = -1)
  /head/mcu_status        std_msgs/Int32 bits: 0 IMU, 1 PCA9685, 2 servo enabled, 3 clock sync
  /head/servo_target_rad  std_msgs/Float32 subscription; the servo releases after
                          command_timeout_s without a new finite command.

The IMU site is identity-aligned with the floating base, matching the policy's
identity ``imu_to_body_rotation``. Noise and bias are optional Gaussian
placeholders, not identified sensor characteristics.
"""
from __future__ import annotations

import json
import math
import time
import threading

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32, String

from .world import ImuSnapshot, SimWorld

# Placeholder covariances documented by the body firmware (roll/pitch 1e-3, yaw 1e6 rad^2).
BODY_ORIENTATION_COVARIANCE = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 1e6]


class _ImuNoise:
    """Additive white noise plus a constant gyro bias, all in sensor units."""

    def __init__(self, node: Node, seed: int) -> None:
        node.declare_parameter('gyro_noise_std_rad_s', 0.0)
        node.declare_parameter('accel_noise_std_m_s2', 0.0)
        node.declare_parameter('gyro_bias_rad_s', [0.0, 0.0, 0.0])
        self.gyro_std = float(node.get_parameter('gyro_noise_std_rad_s').value)
        self.accel_std = float(node.get_parameter('accel_noise_std_m_s2').value)
        self.gyro_bias = np.asarray(node.get_parameter('gyro_bias_rad_s').value, dtype=float)
        if self.gyro_bias.shape != (3,):
            raise ValueError('gyro_bias_rad_s must have three elements')
        self._rng = np.random.default_rng(seed)

    def apply(self, sample: ImuSnapshot) -> tuple[np.ndarray, np.ndarray]:
        gyro = sample.gyro + self.gyro_bias
        accel = sample.accel
        if self.gyro_std > 0.0:
            gyro = gyro + self._rng.normal(0.0, self.gyro_std, 3)
        if self.accel_std > 0.0:
            accel = accel + self._rng.normal(0.0, self.accel_std, 3)
        return gyro, accel


def _fill_imu(msg: Imu, gyro: np.ndarray, accel: np.ndarray) -> None:
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)


class SimBodyImuNode(Node):

    def __init__(self, world: SimWorld) -> None:
        super().__init__('sim_body_imu')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('seed', 1)
        self.world = world
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._noise = _ImuNoise(self, int(self.get_parameter('seed').value))
        self._published = 0
        self.first_sample = threading.Event()
        self._invalid_reported_at = -math.inf
        # The firmware publishes Reliable/Volatile.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.status_pub = self.create_publisher(String, '/imu/status', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._publish)
        self.create_timer(1.0, self._publish_status)

    def _publish(self) -> None:
        if self.world.control_fault is not None:
            return  # Do not re-stamp a frozen physical sample as fresh.
        stamp = self.get_clock().now().to_msg()
        sample = self.world.body_imu()
        gyro, accel = self._noise.apply(sample)
        if (not np.isfinite(gyro).all() or not np.isfinite(accel).all()
                or not np.isfinite(sample.quat_wxyz).all()
                or not np.isclose(np.linalg.norm(sample.quat_wxyz), 1.0, atol=0.01)):
            if time.monotonic() - self._invalid_reported_at > 1.0:
                self._invalid_reported_at = time.monotonic()
                self.get_logger().error('Invalid simulated IMU sample; startup not ready')
            return
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        w, x, y, z = (float(v) for v in sample.quat_wxyz)
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.orientation_covariance = BODY_ORIENTATION_COVARIANCE
        _fill_imu(msg, gyro, accel)
        self.imu_pub.publish(msg)
        self._published += 1
        self.first_sample.set()

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(dict(
            simulated=True, sensor='mujoco', frame_id=self._frame_id,
            published=self._published, subscribers=self.imu_pub.get_subscription_count(),
            clock_synced=True))))


class SimHeadMcuNode(Node):

    STATUS_IMU = 1 << 0
    STATUS_PCA9685 = 1 << 1
    STATUS_SERVO_ENABLED = 1 << 2
    STATUS_CLOCK_SYNC = 1 << 3

    def __init__(self, world: SimWorld) -> None:
        super().__init__('sim_head_mcu')
        self.declare_parameter('imu_rate_hz', 100.0)
        self.declare_parameter('frame_id', 'head_imu_link')
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('seed', 2)
        self.world = world
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._noise = _ImuNoise(self, int(self.get_parameter('seed').value))
        self._last_command = -math.inf
        self.imu_pub = self.create_publisher(Imu, '/head/imu', 10)
        self.status_pub = self.create_publisher(Int32, '/head/mcu_status', 10)
        self.create_subscription(Float32, '/head/servo_target_rad', self._command, 10)
        self.create_timer(1.0 / float(self.get_parameter('imu_rate_hz').value), self._publish_imu)
        # Fast enough to release the servo within the timeout plus one period.
        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0, self._publish_status)

    def _command(self, msg: Float32) -> None:
        if not math.isfinite(msg.data):
            self.world.set_head_target(None)
            return
        self._last_command = time.monotonic()
        self.world.set_head_target(float(msg.data))

    def _servo_enabled(self) -> bool:
        return time.monotonic() - self._last_command <= self._timeout

    def _watchdog(self) -> None:
        if not self._servo_enabled():
            self.world.set_head_target(None)

    def _publish_imu(self) -> None:
        if self.world.control_fault is not None:
            return
        gyro, accel = self._noise.apply(self.world.head_imu())
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation_covariance[0] = -1.0  # No fused orientation on the head IMU.
        _fill_imu(msg, gyro, accel)
        self.imu_pub.publish(msg)

    def _publish_status(self) -> None:
        bits = self.STATUS_IMU | self.STATUS_PCA9685 | self.STATUS_CLOCK_SYNC
        if self._servo_enabled():
            bits |= self.STATUS_SERVO_ENABLED
        self.status_pub.publish(Int32(data=bits))
