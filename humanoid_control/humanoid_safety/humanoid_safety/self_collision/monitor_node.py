#!/usr/bin/env python3
"""Advisory self-collision monitor for measured and policy target leg poses.

Warnings only: this node publishes diagnostics and logs, and never publishes
commands or an estop. It runs in its own process, keeps only the latest
sample from each topic, and checks them on a timer, so it cannot block the
CAN node or the policy. It is deliberately not part of the latched
SafetySupervisor, which would turn a warning into an estop.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ..joint_state_monitor_node import valid_joint_state
from .checker import (
    PoseSample,
    SelfCollisionChecker,
    create_backend,
    evaluate,
    parse_link_pairs,
    srdf_disabled_link_pairs,
    urdf_adjacent_link_pairs,
)


class SelfCollisionMonitorNode(Node):
    def __init__(self):
        super().__init__('self_collision_monitor_node')
        self.declare_parameter('backend', 'pinocchio')
        # Empty paths use the installed humanoid_leg_description URDF.
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('package_dir', '')
        # Optional MoveIt SRDF whose <disable_collisions> pairs are also exempt.
        self.declare_parameter('srdf_path', '')
        # Comma-separated "link_a:link_b" pairs; document why each is allowed.
        self.declare_parameter('extra_allowed_pairs', '')
        self.declare_parameter('measured_topic', '/joint_states')
        self.declare_parameter('target_topic', '/policy/target_angles')
        self.declare_parameter('status_topic', '/safety/self_collision')
        self.declare_parameter('check_rate_hz', 20.0)
        self.declare_parameter('input_timeout_s', 0.1)
        self.declare_parameter('clearance_margin_m', 0.01)
        self.declare_parameter('max_joint_step_rad', 0.05)
        self.declare_parameter('max_path_poses', 100)

        param = lambda name: self.get_parameter(name).value
        self.timeout = float(param('input_timeout_s'))
        self.max_step = float(param('max_joint_step_rad'))
        self.max_path_poses = int(param('max_path_poses'))
        rate = float(param('check_rate_hz'))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('input_timeout_s must be finite and positive')
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('check_rate_hz must be finite and positive')
        self.period = 1.0 / rate

        urdf_path = str(param('urdf_path'))
        package_dir = str(param('package_dir'))
        if not urdf_path or not package_dir:
            share = get_package_share_directory('humanoid_leg_description')
            urdf_path = urdf_path or os.path.join(share, 'urdf', 'human_offset_corrected.urdf')
            # package:// mesh URIs resolve relative to the directory holding the package.
            package_dir = package_dir or os.path.dirname(share)
        allowed = urdf_adjacent_link_pairs(urdf_path) | parse_link_pairs(str(param('extra_allowed_pairs')))
        if param('srdf_path'):
            allowed |= srdf_disabled_link_pairs(str(param('srdf_path')))

        backend = create_backend(str(param('backend')), urdf_path, [package_dir], allowed)
        self.checker = SelfCollisionChecker(backend, float(param('clearance_margin_m')))
        self.measured = None
        self.target = None

        self.status_pub = self.create_publisher(DiagnosticArray, str(param('status_topic')), 10)
        self.create_subscription(JointState, str(param('measured_topic')),
                                 lambda msg: self.on_joint_state('measured', msg), qos_profile_sensor_data)
        self.create_subscription(JointState, str(param('target_topic')),
                                 lambda msg: self.on_joint_state('target', msg), qos_profile_sensor_data)
        self.create_timer(self.period, self.check)
        self.get_logger().info(
            f'Self-collision monitor (advisory): backend={backend.name}, '
            f'{len(backend.link_pairs)} link pairs, margin={self.checker.margin_m} m, '
            f'measured={param("measured_topic")}, target={param("target_topic")}')

    def on_joint_state(self, kind, msg):
        if not valid_joint_state(msg):
            self.get_logger().warn(f'Dropped invalid {kind} JointState', throttle_duration_sec=5.0)
            return
        # Keep only the latest sample; nothing queues behind a slow check.
        sample = PoseSample(
            positions=dict(zip(msg.name, msg.position)),
            received_at=time.monotonic(),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        setattr(self, kind, sample)

    def check(self):
        started = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        reports = evaluate(self.checker, self.measured, self.target, started, now_ros,
                           self.timeout, self.max_step, self.max_path_poses)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self.to_status(report) for report in reports]
        self.status_pub.publish(array)
        # One log call: throttling is per call site, so separate calls per
        # check would hide later checks' warnings behind the first.
        warnings = [f'{status.name.rsplit("/", 1)[-1]}: {status.message}' for status in array.status
                    if status.level in (DiagnosticStatus.WARN, DiagnosticStatus.ERROR)]
        if warnings:
            self.get_logger().warn('Self-collision (advisory only): ' + '; '.join(warnings),
                                   throttle_duration_sec=1.0)
        elapsed = time.monotonic() - started
        if elapsed > self.period:
            self.get_logger().warn(f'Self-collision check took {elapsed * 1000:.1f} ms, '
                                   f'longer than its {self.period * 1000:.1f} ms period',
                                   throttle_duration_sec=5.0)

    def to_status(self, report):
        status = DiagnosticStatus(name=f'humanoid_safety/self_collision/{report.check}',
                                  hardware_id=self.checker.backend.name)
        values = {'margin_m': self.checker.margin_m}
        if report.input_age_s is not None:
            values['input_age_ms'] = report.input_age_s * 1000
        result = report.result
        if result is None:
            status.level = DiagnosticStatus.STALE
            status.message = report.problem
        else:
            pair = '-'.join(result.closest_pair)
            sampled = f'sampled path ({result.poses_checked} poses), ' if report.check == 'path' else ''
            if result.level == 'collision':
                status.level = DiagnosticStatus.ERROR
                status.message = f'{sampled}collision: {pair} ({result.min_distance_m:+.4f} m)'
            elif result.level == 'close':
                status.level = DiagnosticStatus.WARN
                status.message = f'{sampled}below margin: {pair} ({result.min_distance_m:.4f} m)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'{sampled}clear: min {result.min_distance_m:.4f} m ({pair})'
            values.update(
                min_distance_m=result.min_distance_m,
                closest_pair=pair,
                colliding_pairs=' '.join('-'.join(p) for p in result.colliding_pairs),
                close_pairs=' '.join('-'.join(p) for p in result.close_pairs),
                poses_checked=result.poses_checked,
                compute_ms=result.compute_s * 1000)
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        return status


def main(args=None):
    rclpy.init(args=args)
    node = SelfCollisionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
