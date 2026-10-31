#!/usr/bin/env python3
"""Run the leg tracking ONNX at 50 Hz on ROS topics, without an SDK or CAN."""
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import threading
from functools import wraps

from builtin_interfaces.msg import Time

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from motor_control_interfaces.msg import MotorCommand

from motor_control_hybrid.tracking_onnx import (
    GravityEstimator, TrackingPolicy, fresh, joint_feedback, validate_tracking_registry, apply_runtime_policy)


def synchronized(method):
    """Serialize short state operations; never hold this lock across inference."""
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.state_lock:
            return method(self, *args, **kwargs)
    return call


class TrackingPolicyNode(Node):
    def __init__(self):
        super().__init__('tracking_policy_node')
        default_model = str(Path(get_package_share_directory('motor_control_hybrid')) /
                            'models/20260914_170827/20260914_170827.onnx')
        defaults = {
            'model_path': default_model, 'control_rate_hz': 50.0,
            'motor_config_file': '',
            'imu_topic': '/imu/data_raw', 'joint_states_topic': '/joint_states',
            'joint_feedback_mode': 'zero', 'input_timeout_s': 0.1,
            'calibration_seconds': 2.0, 'imu_frame': 'imu_link',
            'max_input_skew_s': 0.02, 'max_inference_s': 0.02,
            'orientation_source': 'message',
            'hold_first_frame': False,
            'reference_end_behavior': 'hold_last_frame', 'wait_for_sim_release': False,
            'reference_loop_start_frame': 50, 'reference_loop_end_frame': 251,
            'imu_to_body_rotation': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.rate = float(param('control_rate_hz'))
        self.timeout = float(param('input_timeout_s'))
        self.max_input_skew = float(param('max_input_skew_s'))
        self.max_inference = float(param('max_inference_s'))
        if not (0 < self.max_input_skew <= self.timeout and 0 < self.max_inference <= 1 / self.rate):
            raise ValueError('Input skew must fit input timeout; inference budget must fit one policy period')
        self.state_lock = threading.RLock()
        self.stopping = threading.Event()
        self.calibration_seconds = float(param('calibration_seconds'))
        # The reference and previous-action observations were trained at 50 Hz.
        if self.rate != 50.0 or not 0 < self.timeout <= 1.0:
            raise ValueError('This tracking export requires 50 Hz and timeout in (0, 1] s')
        if not np.isfinite(self.calibration_seconds) or not 0 <= self.calibration_seconds <= 10:
            raise ValueError('calibration_seconds must be in [0, 10]')
        self.feedback_mode = str(param('joint_feedback_mode'))
        if self.feedback_mode not in ('zero', 'joint_states'):
            raise ValueError('joint_feedback_mode must be zero or joint_states')
        self.orientation_source = str(param('orientation_source'))
        self.hold_first_frame = bool(param('hold_first_frame'))
        if self.orientation_source not in ('complementary', 'message'):
            raise ValueError('orientation_source must be complementary or message')
        self.imu_frame = str(param('imu_frame'))
        self.rotation = np.asarray(param('imu_to_body_rotation')).reshape(3, 3)
        if (not np.allclose(self.rotation @ self.rotation.T, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(self.rotation), 1.0)):
            raise ValueError('imu_to_body_rotation must be a proper orthonormal rotation')
        model = Path(str(param('model_path'))).expanduser()
        self.policy = TrackingPolicy(model)
        self.model_sha256 = hashlib.sha256(model.read_bytes()).hexdigest()
        registry_path = str(param('motor_config_file'))
        if registry_path:
            params = yaml.safe_load(Path(registry_path).read_text())['motor_control_node']['ros__parameters']
            validate_tracking_registry(self.policy, params, self.model_sha256)
            apply_runtime_policy(self.policy, params)
        self.previous = np.zeros(10, dtype=np.float32)
        for frame in range(30):
            obs = self.policy.observation(frame % self.policy.frames, np.zeros(3),
                                          np.array([0, 0, -1]), self.policy.offset,
                                          np.zeros(10), self.previous)
            self.policy.run(obs, frame % self.policy.frames)
        self.reference_end_behavior = str(param('reference_end_behavior'))
        self.loop_start = int(param('reference_loop_start_frame'))
        self.loop_end = int(param('reference_loop_end_frame'))
        if self.reference_end_behavior not in ('hold_last_frame', 'loop'):
            raise ValueError('reference_end_behavior must be hold_last_frame or loop')
        if self.reference_end_behavior == 'loop' and not 0 <= self.loop_start < self.loop_end <= self.policy.frames:
            raise ValueError('Reference loop must fit the exported reference frames')
        self.reference_cycles = 0
        self.wait_for_sim_release = bool(param('wait_for_sim_release'))
        self.reference_started = None
        self.filter = GravityEstimator() if self.orientation_source == 'complementary' else None
        self.bias = np.zeros(3)
        self.calibration = []
        self.calibration_start = None
        self.calibrated = self.calibration_seconds == 0
        self.imu = self.joints = None
        self.estop = False
        self.fault = None
        self.expiry_diagnostic = None
        self.started = None
        self.state = 'waiting_for_imu'
        self.count = self.overruns = 0
        self.rejected_imu_timestamps = 0
        self.last_tick = None
        self.intervals = deque(maxlen=3000)
        self.inference_ms = deque(maxlen=3000)
        self.work_ms = deque(maxlen=3000)
        self.frame = 0
        stream_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.angles_pub = self.create_publisher(JointState, '/policy/target_angles', stream_qos)
        self.actions_pub = self.create_publisher(Float64MultiArray, '/policy/actions', stream_qos)
        # A monitor topic with the existing C++ scheduler's message contract.
        # The launch never forwards this to a hardware motor topic.
        self.command_pub = self.create_publisher(MotorCommand, '/policy/desired_motor_subset', stream_qos)
        self.status_pub = self.create_publisher(String, '/policy/status', stream_qos)
        self.create_subscription(Imu, str(param('imu_topic')), self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, str(param('joint_states_topic')),
                                 self.on_joints, 1)
        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, estop_qos)
        if self.wait_for_sim_release:
            self.create_subscription(String, '/sim/status', self.on_sim_status, 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Single inference worker, no pending FIFO: every iteration snapshots latest inputs.
        self.worker = threading.Thread(target=self.run_worker, name='tracking-inference', daemon=True)
        self.worker.start()
        self.create_timer(1.0, self.publish_status, clock=self.steady_clock)
        self.get_logger().info(
            f'ONNX tracking: 50 Hz, feedback={self.feedback_mode}, orientation={self.orientation_source}, '
            f'reference={"hold_first_frame" if self.hold_first_frame else "play"}, '
            'outputs=/policy/*; no SDK, CAN or motor enable commands. '
            'Keep the IMU stationary during calibration.')

    @synchronized
    def on_sim_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if status.get('base_held') is False and self.reference_started is None:
            self.reference_started = time.monotonic()

    def reference_frame(self, index):
        if self.hold_first_frame:
            return 0, 0
        if self.reference_end_behavior == 'loop' and index >= self.loop_end:
            length = self.loop_end - self.loop_start
            return (self.loop_start + (index - self.loop_start) % length,
                    (index - self.loop_start) // length)
        return min(index, self.policy.frames - 1), 0

    @staticmethod
    def stamp_seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @synchronized
    def latch_fault(self, reason, details=None):
        if self.stopping.is_set():
            return
        if self.fault is None:
            detail = '' if details is None else '; ' + json.dumps(details, allow_nan=False)
            self.get_logger().error(reason + detail + '; restart the node to reset')
        self.fault = reason
        self.previous[:] = 0

    @synchronized
    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.latch_fault('estop')

    @synchronized
    def on_joints(self, msg):
        try:
            q, dq = joint_feedback(msg.name, msg.position, msg.velocity, self.policy.names)
            self.joints = (time.monotonic(), self.stamp_seconds(msg), q, dq)
        except ValueError as exc:
            self.joints = None
            if self.started is not None and self.feedback_mode == 'joint_states':
                self.latch_fault(str(exc))

    @synchronized
    def on_imu(self, msg):
        if self.fault is not None:
            return
        now = time.monotonic()
        stamp = self.stamp_seconds(msg)
        # DDS discovery can deliver old queued samples at startup. Reject them
        # without replacing the last valid sample; the control watchdog checks
        # that a current sample actually arrives before any inference continues.
        if (stamp <= 0 or not fresh(now, stamp, now, self.ros_seconds(), self.timeout)
                or (self.imu is not None and stamp <= self.imu[1])):
            self.rejected_imu_timestamps += 1
            return
        try:
            if msg.header.frame_id != self.imu_frame:
                raise ValueError('IMU frame does not match imu_frame parameter')
            acc = self.rotation @ np.array([msg.linear_acceleration.x,
                                           msg.linear_acceleration.y, msg.linear_acceleration.z])
            gyro = self.rotation @ np.array([msg.angular_velocity.x,
                                            msg.angular_velocity.y, msg.angular_velocity.z])
            if not np.isfinite(acc).all() or not np.isfinite(gyro).all():
                raise ValueError('Non-finite IMU sample')
            if not 0.1 < np.linalg.norm(acc) < 100:
                raise ValueError('Invalid IMU acceleration')
            if not self.calibrated:
                if self.calibration_start is None:
                    self.calibration_start = now
                self.calibration.append((acc, gyro))
                if now - self.calibration_start < self.calibration_seconds:
                    self.state = 'calibrating'
                    return
                accelerations, velocities = np.asarray(self.calibration).transpose(1, 0, 2)
                if (len(velocities) < 20 or np.max(velocities.std(axis=0)) > 0.03
                        or np.linalg.norm(velocities.mean(axis=0)) > 0.15
                        or not np.all((np.linalg.norm(accelerations, axis=1) > 8.8)
                                      & (np.linalg.norm(accelerations, axis=1) < 10.8))):
                    raise ValueError('IMU was not stationary during startup calibration')
                self.bias = velocities.mean(axis=0)
                self.calibration.clear()
                self.calibrated = True
            omega = gyro - self.bias
            if self.orientation_source == 'message':
                quat = np.array([msg.orientation.w, msg.orientation.x,
                                 msg.orientation.y, msg.orientation.z])
                if (msg.orientation_covariance[0] < 0 or not np.isfinite(quat).all()
                        or not np.isclose(np.linalg.norm(quat), 1, atol=0.01)):
                    raise ValueError('IMU orientation is unavailable or invalid')
                w, x, y, z = quat / np.linalg.norm(quat)
                # Inverse world<-sensor rotation applied to world down.
                gravity = self.rotation @ np.array([
                    2 * (w*y - x*z), -2 * (y*z + w*x), 2 * (x*x + y*y) - 1])
            else:
                gravity = self.filter.update(acc, omega, stamp)
            self.imu = (now, stamp, omega, gravity)
        except ValueError as exc:
            self.imu = None
            self.latch_fault(str(exc))

    def run_worker(self):
        next_tick = time.monotonic()
        while not self.stopping.is_set():
            self.tick()
            next_tick += 1 / self.rate
            now = time.monotonic()
            # Missed periods are skipped, never replayed in a catch-up burst.
            if next_tick < now:
                next_tick = now + 1 / self.rate
            self.stopping.wait(max(0, next_tick - now))

    def destroy_node(self):
        self.stopping.set()
        self.worker.join(timeout=1.0)
        # A late inference checks stopping before touching a publisher.
        with self.state_lock:
            return super().destroy_node()

    @staticmethod
    def time_message(seconds):
        ns = int(seconds * 1e9)
        sec, nanosec = divmod(ns, 10**9)
        return Time(sec=sec, nanosec=nanosec)

    def tick(self):
        with self.state_lock:
            begin = time.monotonic()
            if self.stopping.is_set() or self.fault or self.estop:
                self.state = 'fault' if self.fault or self.estop else 'stopping'
                return
            if self.imu is None:
                return
            waiting_for_release = self.wait_for_sim_release and self.reference_started is None
            now_ros = self.ros_seconds()
            imu = self.imu  # Callbacks replace tuples and never mutate their arrays.
            joints = self.joints if self.feedback_mode == 'joint_states' else None
            if not fresh(*imu[:2], begin, now_ros, self.timeout):
                if waiting_for_release:
                    self.state = 'waiting_for_fresh_sim_inputs'
                    return
                self.latch_fault('IMU timeout')
                return
            if self.feedback_mode == 'joint_states':
                if joints is None:
                    self.state = 'waiting_for_joint_states'
                    return
                if not fresh(*joints[:2], begin, now_ros, self.timeout):
                    if waiting_for_release:
                        self.state = 'waiting_for_fresh_sim_inputs'
                        return
                    self.latch_fault('JointState timeout')
                    return
                if abs(imu[1] - joints[1]) > self.max_input_skew:
                    self.state = 'waiting_for_aligned_inputs'
                    return
                q, dq = joints[2:]
            else:
                q, dq = self.policy.offset, np.zeros(10, dtype=np.float32)
            if self.started is None and not waiting_for_release:
                self.started = begin
            origin = self.reference_started if self.wait_for_sim_release else self.started
            index = 0 if origin is None else max(0, int((begin - origin) * 50))
            frame, self.reference_cycles = self.reference_frame(index)
            source_stamp = min(imu[1], joints[1]) if joints else imu[1]
            valid_until = source_stamp + self.timeout
            previous = (np.zeros_like(self.previous) if self.wait_for_sim_release and self.reference_started is None
                        else self.previous.copy())
        try:
            obs = self.policy.observation(frame, *imu[2:], q, dq, previous)
            start_inference = time.monotonic()
            actions, targets = self.policy.run(obs, frame)
            finished = time.monotonic()
            with self.state_lock:
                lock_acquired = time.monotonic()
                self.inference_ms.append((finished - start_inference) * 1000)
                if self.stopping.is_set() or self.fault or self.estop:
                    return
                # Validate the immutable inputs used by this result, not newer callbacks.
                checked_at = time.monotonic()
                checked_ros = self.ros_seconds()
                elapsed_expired = checked_at - begin > self.max_inference
                imu_fresh = fresh(*imu[:2], checked_at, checked_ros, self.timeout)
                joints_fresh = (joints is None or fresh(
                    *joints[:2], checked_at, checked_ros, self.timeout))
                if elapsed_expired or not imu_fresh or not joints_fresh:
                    # Diagnose the rejected immutable snapshot, never newer callbacks.
                    self.expiry_diagnostic = {
                        'causes': [name for name, failed in (
                            ('control_budget', elapsed_expired), ('imu_stale', not imu_fresh),
                            ('joints_stale', not joints_fresh)) if failed],
                        'elapsed_ms': (checked_at - begin) * 1000,
                        'prepare_ms': (start_inference - begin) * 1000,
                        'inference_ms': (finished - start_inference) * 1000,
                        'result_lock_wait_ms': (lock_acquired - finished) * 1000,
                        'budget_ms': self.max_inference * 1000,
                        'input_timeout_ms': self.timeout * 1000,
                        'imu_source_age_ms': (checked_ros - imu[1]) * 1000,
                        'imu_receipt_age_ms': (checked_at - imu[0]) * 1000,
                        'joint_source_age_ms': None if joints is None else
                            (checked_ros - joints[1]) * 1000,
                        'joint_receipt_age_ms': None if joints is None else
                            (checked_at - joints[0]) * 1000,
                        'reference_frame': frame,
                        'hold_first_frame': self.hold_first_frame,
                    }
                    self.latch_fault('Inference result expired', details=self.expiry_diagnostic)
                    return
                if not np.isfinite(actions).all() or not np.isfinite(targets).all():
                    raise ValueError('Non-finite policy output')
                angles = JointState()
                angles.header.stamp = self.time_message(source_stamp)
                angles.header.frame_id = 'policy_body'
                angles.name = self.policy.names
                angles.position = targets.astype(float).tolist()
                command = MotorCommand()
                command.header = angles.header
                command.valid_until = self.time_message(valid_until)
                command.joint_name = self.policy.names
                command.mode = [MotorCommand.MODE_MOTION] * 10
                command.position = angles.position
                command.velocity = [0.0] * 10
                command.acceleration = [0.0] * 10
                command.torque = [0.0] * 10
                command.kp = self.policy.kp.astype(float).tolist()
                command.kd = self.policy.kd.astype(float).tolist()
                self.command_pub.publish(command)
                self.angles_pub.publish(angles)
                self.actions_pub.publish(Float64MultiArray(data=actions.astype(float).tolist()))
                # Rejected/expired results must never advance previous-action history.
                self.previous = actions.copy()
                self.frame = frame
                self.count += 1
                if self.last_tick is not None:
                    self.intervals.append((begin - self.last_tick) * 1000)
                self.last_tick = begin
                work_ms = (time.monotonic() - begin) * 1000
                self.work_ms.append(work_ms)
                self.overruns += int(work_ms > 1000 / self.rate)
                self.state = 'running'
        except Exception as exc:
            # A dead worker must not silently leave its last target running.
            self.latch_fault(f'Policy worker: {exc}')

    @staticmethod
    def statistics(values):
        if not values:
            return None
        return {'mean': float(np.mean(values)), 'p99': float(np.percentile(values, 99)),
                'max': float(np.max(values))}

    def publish_status(self):
        # Copy bounded histories under lock; percentile/JSON/DDS work must not
        # hold up feedback callbacks or the inference worker.
        with self.state_lock:
            if self.stopping.is_set():
                return
            now = time.monotonic()
            status = {
                'state': self.state, 'fault': self.fault, 'requested_hz': self.rate,
                'expiry_diagnostic': self.expiry_diagnostic,
                'max_input_skew_s': self.max_input_skew, 'max_inference_s': self.max_inference,
                'iterations': self.count, 'reference_frame': self.frame,
                'reference_finished': (self.reference_cycles > 0 if self.reference_end_behavior == 'loop'
                                       else self.frame == self.policy.frames - 1),
                'reference_cycles': self.reference_cycles,
                'waiting_for_sim_release': self.wait_for_sim_release and self.reference_started is None,
                'hold_first_frame': self.hold_first_frame,
                'reference_end_behavior': 'hold_first_frame' if self.hold_first_frame else self.reference_end_behavior,
                'joint_feedback': self.feedback_mode, 'hardware_output': False,
                'imu_age_since_receipt_ms': None if self.imu is None else (now - self.imu[0]) * 1000,
                'gyro_bias_rad_s': self.bias.tolist(), 'model_sha256': self.model_sha256,
                'rejected_imu_timestamps': self.rejected_imu_timestamps,
                'inference_ms': None,
                'control_callback_ms': None,
                'tick_interval_ms': None,
                'callback_overruns_20ms': self.overruns,
            }
            timing_samples = {
                'inference_ms': tuple(self.inference_ms),
                'control_callback_ms': tuple(self.work_ms),
                'tick_interval_ms': tuple(self.intervals),
            }
        status.update({name: self.statistics(samples) for name, samples in timing_samples.items()})
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
