"""Simulated micro-ROS MCUs: the body BNO085 IMU and the head IMU/servo board.

Body IMU (replaces policy_imu_microros firmware + micro_ros_agent):
  /imu/data_raw  sensor_msgs/Imu, frame imu_link, 100 Hz
                 orientation    sensor-to-world quaternion (like Game RV; yaw is
                                relative to the spawn heading, not a compass;
                                optional periodic heading error, see _YawError)
                 angular_vel    rad/s in the sensor frame
                 linear_accel   specific force in m/s^2, ~[0, 0, +9.81] upright
  /imu/status    std_msgs/String JSON, 1 Hz

Head MCU (replaces humanoid_vision_microros firmware + micro_ros_agent):
  /head/imu               sensor_msgs/Imu without orientation (covariance[0] = -1)
  /head/mcu_status        std_msgs/Int32 bits: 0 IMU, 1 PCA9685, 2 servo enabled, 3 clock sync
  /head/servo_target_rad  std_msgs/Float32 subscription; the servo releases after
                          command_timeout_s without a new finite command.

The optional two-axis scene also accepts /head/tilt_target_rad, with an
independent timeout after its first finite pitch command.

The body IMU site is identity-aligned with the floating base, matching the policy's
identity ``imu_to_body_rotation``. Noise and bias are optional Gaussian
placeholders, not identified sensor characteristics.
"""
from __future__ import annotations

import json
import math
import contextlib
import gc
import time
import threading

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32, String

from .world import ImuSnapshot, SimWorld
from .publish_sample import publish_sample

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


class _YawError:
    """Periodic heading error of the Game Rotation Vector (no magnetometer).

    The published attitude is the true one turned about world z by
    ``amplitude * sin(2*pi*t / period)``, t from the first sample. Roll, pitch,
    gyro and accelerometer are unchanged. Amplitude 0 (default) disables it.
    """

    def __init__(self, node: Node) -> None:
        node.declare_parameter('yaw_error_amplitude_deg', 0.0)
        node.declare_parameter('yaw_error_period_s', 60.0)
        self.amplitude = math.radians(float(node.get_parameter('yaw_error_amplitude_deg').value))
        self.period = float(node.get_parameter('yaw_error_period_s').value)
        if self.amplitude != 0.0 and not self.period > 0.0:
            raise ValueError('yaw_error_period_s must be positive')
        self._start_ns = None

    def error_rad(self, stamp_ns: int) -> float:
        if self.amplitude == 0.0:
            return 0.0
        if self._start_ns is None:
            self._start_ns = stamp_ns
        t = (stamp_ns - self._start_ns) * 1e-9
        return self.amplitude * math.sin(2.0 * math.pi * t / self.period)

    def apply(self, quat_wxyz, stamp_ns: int) -> np.ndarray:
        q = np.asarray(quat_wxyz, dtype=float)
        e = self.error_rad(stamp_ns)
        if e == 0.0:
            return q
        c, s = math.cos(e / 2.0), math.sin(e / 2.0)
        w, x, y, z = q  # (c, 0, 0, s) * q: rotation about world z applied after q.
        return np.array([c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w])


def _fill_imu(msg: Imu, gyro: np.ndarray, accel: np.ndarray) -> None:
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)


class SimBodyImuNode(Node):

    def __init__(self, world: SimWorld, diagnostics_node=None) -> None:
        super().__init__('sim_body_imu')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('seed', 1)
        self.world = world
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._noise = _ImuNoise(self, int(self.get_parameter('seed').value))
        self._yaw_error = _YawError(self)
        self._published = 0
        self._last_publish_mono = None
        self._max_publish_gap_s = 0.0
        self._max_sample_wait_s = 0.0
        self._last_callback_mono = None
        self._publish_thread = None
        self._publish_native_tid = None
        self._max_callback_gap_s = 0.0
        self._max_publish_call_s = 0.0
        self._max_status_publish_s = 0.0
        self.declare_parameter('gc_diagnostics', False)
        self._max_gc_pause_s = 0.0
        self._gc_callback = None
        if self.get_parameter('gc_diagnostics').value:
            gc_begin = [0.0]
            def measure_gc(phase, info):
                if phase == 'start':
                    gc_begin[0] = time.monotonic()
                elif phase == 'stop':
                    self._max_gc_pause_s = max(self._max_gc_pause_s, time.monotonic()-gc_begin[0])
            self._gc_callback = measure_gc
            gc.callbacks.append(measure_gc)
        self.first_sample = threading.Event()
        self._invalid_reported_at = -math.inf
        # The firmware publishes Reliable/Volatile.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.status_pub = self.create_publisher(String, '/imu/status', 2)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._publish)
        (diagnostics_node if diagnostics_node is not None else self).create_timer(1.0, self._publish_status)

    def destroy_node(self):
        if self._gc_callback is not None and self._gc_callback in gc.callbacks:
            gc.callbacks.remove(self._gc_callback)
        return super().destroy_node()

    def _publish(self) -> None:
        self._publish_thread = threading.current_thread().name
        self._publish_native_tid = threading.get_native_id()
        entered = time.monotonic()
        if self._last_callback_mono is not None:
            self._max_callback_gap_s = max(self._max_callback_gap_s, entered-self._last_callback_mono)
        self._last_callback_mono = entered
        recorder = getattr(self.world, 'recorder', None)
        # Collection: keep recorded truth and the published noisy IMU at the same step.
        with self.world.lock if recorder is not None else contextlib.nullcontext():
            if self.world.control_fault is not None:
                return  # Do not re-stamp a frozen physical sample as fresh.
            sample_begin = time.monotonic()
            sample = self.world.body_imu()
            self._max_sample_wait_s = max(self._max_sample_wait_s, time.monotonic() - sample_begin)
            gyro, accel = self._noise.apply(sample)
            if (not np.isfinite(gyro).all() or not np.isfinite(accel).all()
                    or not np.isfinite(sample.quat_wxyz).all()
                    or not np.isclose(np.linalg.norm(sample.quat_wxyz), 1.0, atol=0.01)):
                if time.monotonic() - self._invalid_reported_at > 1.0:
                    self._invalid_reported_at = time.monotonic()
                    self.get_logger().error('Invalid simulated IMU sample; startup not ready')
                return
            msg = Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(sample.stamp_ns, 10**9)
            msg.header.frame_id = self._frame_id
            w, x, y, z = (float(v) for v in self._yaw_error.apply(sample.quat_wxyz, sample.stamp_ns))
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
            msg.orientation_covariance = BODY_ORIENTATION_COVARIANCE
            _fill_imu(msg, gyro, accel)
            publish_begin = time.monotonic()
            try:
                sent = publish_sample(self, self.imu_pub, msg, '/imu/data_raw')
            finally:
                self._max_publish_call_s = max(self._max_publish_call_s, time.monotonic()-publish_begin)
            if not sent:
                return
            if recorder is not None:
                recorder.record_imu(sample, gyro, accel, msg.header.stamp)
            published_at = time.monotonic()
            if self._last_publish_mono is not None:
                self._max_publish_gap_s = max(self._max_publish_gap_s,
                                              published_at - self._last_publish_mono)
            self._last_publish_mono = published_at
            self._published += 1
            self.first_sample.set()

    def _publish_status(self) -> None:
        begin = time.monotonic()
        publish_sample(self, self.status_pub, String(data=json.dumps(dict(
            simulated=True, sensor='mujoco', frame_id=self._frame_id,
            published=self._published, subscribers=self.imu_pub.get_subscription_count(),
            publish_thread=self._publish_thread, publish_native_tid=self._publish_native_tid,
            qos_depth=self.imu_pub.qos_profile.depth,
            last_publish_age_s=(None if self._last_publish_mono is None else
                                time.monotonic() - self._last_publish_mono),
            max_publish_gap_s=self._max_publish_gap_s,
            max_sample_wait_s=self._max_sample_wait_s,
            max_callback_gap_s=self._max_callback_gap_s,
            max_publish_call_s=self._max_publish_call_s,
            max_status_publish_s=self._max_status_publish_s,
            max_gc_pause_s=self._max_gc_pause_s,
            publish_failures=dict(getattr(self, '_publish_failures', {})),
            control_fault=self.world.control_fault,
            clock_synced=True))), '/imu/status')
        self._max_status_publish_s = max(self._max_status_publish_s, time.monotonic()-begin)


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
        self._last_pitch_command = None
        self.imu_pub = self.create_publisher(Imu, '/head/imu', 2)
        self.status_pub = self.create_publisher(Int32, '/head/mcu_status', 2)
        self.create_subscription(Float32, '/head/servo_target_rad', self._command, 2)
        if world._pitch_joint >= 0:
            self.create_subscription(Float32, '/head/tilt_target_rad', self._pitch_command, 2)
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

    def _pitch_command(self, msg: Float32) -> None:
        if not math.isfinite(msg.data):
            self.world.set_pitch_target(None)
            return
        self._last_pitch_command = time.monotonic()
        self.world.set_pitch_target(float(msg.data))

    def _watchdog(self) -> None:
        if not self._servo_enabled():
            self.world.set_head_target(None)
        if (self._last_pitch_command is not None and
                time.monotonic() - self._last_pitch_command > self._timeout):
            self.world.set_pitch_target(None)

    def _publish_imu(self) -> None:
        if self.world.control_fault is not None:
            return
        gyro, accel = self._noise.apply(self.world.head_imu())
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation_covariance[0] = -1.0  # No fused orientation on the head IMU.
        _fill_imu(msg, gyro, accel)
        publish_sample(self, self.imu_pub, msg, '/head/imu')

    def _publish_status(self) -> None:
        bits = self.STATUS_IMU | self.STATUS_PCA9685 | self.STATUS_CLOCK_SYNC
        if self._servo_enabled():
            bits |= self.STATUS_SERVO_ENABLED
        publish_sample(self, self.status_pub, Int32(data=bits), '/head/mcu_status')
