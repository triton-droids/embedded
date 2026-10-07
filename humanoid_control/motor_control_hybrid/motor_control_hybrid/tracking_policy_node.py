#!/usr/bin/env python3
"""Run the leg tracking ONNX at 50 Hz on ROS topics, without an SDK or CAN."""
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from motor_control_interfaces.msg import MotorCommand

from motor_control_hybrid.tracking_onnx import (
    GravityEstimator, TrackingPolicy, fresh, joint_feedback)


class TrackingPolicyNode(Node):
    def __init__(self):
        super().__init__('tracking_policy_node')
        default_model = str(Path.home() / 'Github/simulation/logs/legs_tracking/'
                            '20260914_170827/20260914_170827.onnx')
        defaults = {
            'model_path': default_model, 'control_rate_hz': 50.0,
            'imu_topic': '/imu/data_raw', 'joint_states_topic': '/joint_states',
            'joint_feedback_mode': 'zero', 'input_timeout_s': 0.1,
            'calibration_seconds': 2.0, 'imu_frame': 'imu_link',
            'orientation_source': 'complementary',
            'imu_to_body_rotation': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.rate = float(param('control_rate_hz'))
        self.timeout = float(param('input_timeout_s'))
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
        self.previous = np.zeros(10, dtype=np.float32)
        for frame in range(30):
            obs = self.policy.observation(frame % self.policy.frames, np.zeros(3),
                                          np.array([0, 0, -1]), self.policy.offset,
                                          np.zeros(10), self.previous)
            self.policy.run(obs, frame % self.policy.frames)
        self.filter = GravityEstimator()
        self.bias = np.zeros(3)
        self.calibration = []
        self.calibration_start = None
        self.calibrated = self.calibration_seconds == 0
        self.imu = self.joints = None
        self.estop = False
        self.fault = None
        self.started = None
        self.state = 'waiting_for_imu'
        self.count = self.overruns = 0
        self.rejected_imu_timestamps = 0
        self.last_tick = None
        self.intervals = deque(maxlen=3000)
        self.inference_ms = deque(maxlen=3000)
        self.work_ms = deque(maxlen=3000)
        self.frame = 0
        self.angles_pub = self.create_publisher(JointState, '/policy/target_angles', 1)
        self.actions_pub = self.create_publisher(Float64MultiArray, '/policy/actions', 1)
        # A monitor topic with the existing C++ scheduler's message contract.
        # The launch never forwards this to a hardware motor topic.
        self.command_pub = self.create_publisher(MotorCommand, '/policy/desired_motor_subset', 1)
        self.status_pub = self.create_publisher(String, '/policy/status', 1)
        self.create_subscription(Imu, str(param('imu_topic')), self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, str(param('joint_states_topic')),
                                 self.on_joints, 1)
        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, estop_qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / self.rate, self.tick, clock=self.steady_clock)
        self.create_timer(1.0, self.publish_status, clock=self.steady_clock)
        self.get_logger().info(
            f'ONNX tracking: 50 Hz, feedback={self.feedback_mode}, '
            'outputs=/policy/*; no SDK, CAN or motor enable commands. '
            'Keep the IMU stationary during calibration.')

    @staticmethod
    def stamp_seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def latch_fault(self, reason):
        if self.fault is None:
            self.get_logger().error(reason + '; restart the node to reset')
        self.fault = reason
        self.previous[:] = 0

    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.latch_fault('estop')

    def on_joints(self, msg):
        try:
            q, dq = joint_feedback(msg.name, msg.position, msg.velocity, self.policy.names)
            self.joints = (time.monotonic(), self.stamp_seconds(msg), q, dq)
        except ValueError as exc:
            self.joints = None
            if self.started is not None and self.feedback_mode == 'joint_states':
                self.latch_fault(str(exc))

    def on_imu(self, msg):
        if self.fault is not None:
            return
        now = time.monotonic()
        stamp = self.stamp_seconds(msg)
        # DDS discovery can deliver old queued samples at startup. Reject them
        # without replacing the last valid sample; the control watchdog checks
        # that a current sample actually arrives before any inference continues.
        if not fresh(now, stamp, now, self.ros_seconds(), self.timeout):
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
                gravity = self.filter.update(acc, omega, now)
            self.imu = (now, stamp, omega, gravity)
        except ValueError as exc:
            self.imu = None
            self.latch_fault(str(exc))

    def tick(self):
        begin = time.monotonic()
        if self.fault or self.estop:
            self.state = 'fault'
            return
        if self.imu is None:
            return
        now_ros = self.ros_seconds()
        if not fresh(*self.imu[:2], begin, now_ros, self.timeout):
            self.latch_fault('IMU timeout')
            return
        if self.feedback_mode == 'joint_states':
            if self.joints is None:
                self.state = 'waiting_for_joint_states'
                return
            if not fresh(*self.joints[:2], begin, now_ros, self.timeout):
                self.latch_fault('JointState timeout')
                return
            q, dq = self.joints[2:]
        else:
            q, dq = self.policy.offset, np.zeros(10, dtype=np.float32)
        if self.started is None:
            self.started = begin
        self.frame = min(int((begin - self.started) * 50), self.policy.frames - 1)
        try:
            obs = self.policy.observation(self.frame, *self.imu[2:], q, dq, self.previous)
            start_inference = time.monotonic()
            actions, targets = self.policy.run(obs, self.frame)
            self.inference_ms.append((time.monotonic() - start_inference) * 1000)
            self.previous = actions
            angles = JointState()
            angles.header.stamp = self.get_clock().now().to_msg()
            angles.header.frame_id = 'policy_body'
            angles.name = self.policy.names
            angles.position = targets.astype(float).tolist()
            self.angles_pub.publish(angles)
            self.actions_pub.publish(Float64MultiArray(data=actions.astype(float).tolist()))
            command = MotorCommand()
            command.header = angles.header
            command.joint_name = self.policy.names
            command.mode = [MotorCommand.MODE_MOTION] * 10
            command.position = angles.position
            command.velocity = [0.0] * 10
            command.acceleration = [0.0] * 10
            command.torque = [0.0] * 10
            command.kp = self.policy.kp.astype(float).tolist()
            command.kd = self.policy.kd.astype(float).tolist()
            self.command_pub.publish(command)
            self.count += 1
            if self.last_tick is not None:
                self.intervals.append((begin - self.last_tick) * 1000)
            self.last_tick = begin
            work_ms = (time.monotonic() - begin) * 1000
            self.work_ms.append(work_ms)
            self.overruns += int(work_ms > 1000 / self.rate)
            self.state = 'running'
        except (ValueError, RuntimeError) as exc:
            self.latch_fault(str(exc))

    @staticmethod
    def statistics(values):
        if not values:
            return None
        return {'mean': float(np.mean(values)), 'p99': float(np.percentile(values, 99)),
                'max': float(np.max(values))}

    def publish_status(self):
        now = time.monotonic()
        status = {
            'state': self.state, 'fault': self.fault, 'requested_hz': self.rate,
            'iterations': self.count, 'reference_frame': self.frame,
            'reference_finished': self.frame == self.policy.frames - 1,
            'reference_end_behavior': 'hold_last_frame',
            'joint_feedback': self.feedback_mode, 'hardware_output': False,
            'imu_age_since_receipt_ms': None if self.imu is None else (now - self.imu[0]) * 1000,
            'gyro_bias_rad_s': self.bias.tolist(), 'model_sha256': self.model_sha256,
            'rejected_imu_timestamps': self.rejected_imu_timestamps,
            'inference_ms': self.statistics(self.inference_ms),
            'control_callback_ms': self.statistics(self.work_ms),
            'tick_interval_ms': self.statistics(self.intervals),
            'callback_overruns_20ms': self.overruns,
        }
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
