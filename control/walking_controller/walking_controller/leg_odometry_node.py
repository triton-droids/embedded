"""Leg odometry node: /imu/data_raw + /joint_states -> odom TF and /odom (see leg_odometry.py).

Publishes the same interface as the simulator's ground-truth ``sim_odom_tf``
(``odom -> base_footprint -> base_link`` on /tf, ``nav_msgs/Odometry`` on
``odom_topic``), so consumers (the ball tracker) run unchanged on the robot.
Uses only sensors the robot has: the body IMU orientation and joint angles.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .leg_odometry import LegOdometry, matrix_to_quat_xyzw
from .robot import RobotModel


def _transform(T, parent, child, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
    t, r = msg.transform.translation, msg.transform.rotation
    t.x, t.y, t.z = (float(v) for v in T[:3, 3])
    r.x, r.y, r.z, r.w = matrix_to_quat_xyzw(T[:3, :3])
    return msg


def _rpy_matrix(roll, pitch, yaw) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (f(a) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class LegOdometryNode(Node):
    def __init__(self):
        super().__init__('leg_odometry')
        p = self.declare_parameter
        urdf = p('urdf_path', '').value or str(
            Path(get_package_share_directory('humanoid_leg_description')) / 'urdf/human_offset_corrected.urdf')
        forward = p('base_forward_axis', 'positive_y').value  # model axis the toes point along
        imu_rpy = p('imu_to_body_rpy', [0.0, 0.0, 0.0]).value  # body <- IMU sensor rotation
        margin = float(p('switch_margin_m', 0.01).value)
        self.odom_frame = p('odom_frame', 'odom').value
        self.footprint_frame = p('base_footprint_frame', 'base_footprint').value
        self.base_frame = p('base_frame', 'base_link').value
        rate = float(p('publish_rate_hz', 50.0).value)
        self.imu_timeout = float(p('imu_timeout_s', 0.1).value)
        odom_topic = p('odom_topic', '/odom').value
        imu_topic = p('imu_topic', '/imu/data_raw').value
        joints_topic = p('joint_states_topic', '/joint_states').value

        self.robot = RobotModel(urdf)
        self.odometry = LegOdometry(self.robot, forward, _rpy_matrix(*imu_rpy), margin)
        self.period_ns = int(1e9 / rate)
        self._imu = None  # (quat xyzw, stamp ns)
        self._last_ns = None
        self._last_pose = None  # (stamp ns, odom_from_fp) for the twist
        self._velocity = (np.zeros(3), 0.0)
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10) if odom_topic else None
        self._status_pub = self.create_publisher(String, '/leg_odometry/status', 2)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, imu_topic, self._on_imu, sensor)
        self.create_subscription(JointState, joints_topic, self._on_joints, sensor)
        self._stance = None
        stance_topic = p('stance_topic', '/walk/stance').value
        if stance_topic:
            self.create_subscription(
                String, stance_topic, self._on_stance,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._skipped = {'no_imu': 0, 'stale_imu': 0, 'missing_joints': 0}
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'Leg odometry: {imu_topic} attitude + {joints_topic} kinematics -> '
            f'{self.odom_frame} -> {self.footprint_frame} -> {self.base_frame} at {rate:.0f} Hz')

    @staticmethod
    def _ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _on_imu(self, msg: Imu):
        o = msg.orientation
        if msg.orientation_covariance[0] < 0.0:
            return  # no orientation in this message
        self._imu = ((o.x, o.y, o.z, o.w), self._ns(msg.header.stamp))

    def _on_stance(self, msg: String):
        if msg.data in ('left_foot', 'right_foot'):
            self._stance = msg.data

    def _on_joints(self, msg: JointState):
        stamp_ns = self._ns(msg.header.stamp)
        if self._last_ns is not None and stamp_ns - self._last_ns < self.period_ns:
            return
        if self._imu is None:
            self._skipped['no_imu'] += 1
            return
        quat, imu_ns = self._imu
        if abs(stamp_ns - imu_ns) > self.imu_timeout * 1e9:
            self._skipped['stale_imu'] += 1
            return
        index = {n: i for i, n in enumerate(msg.name)}
        try:
            joints = [msg.position[index[n]] for n in self.robot.joint_names]
        except (KeyError, IndexError):
            self._skipped['missing_joints'] += 1
            return
        try:
            est = self.odometry.update(joints, quat, self._stance)
        except ValueError as exc:
            self.get_logger().warn(f'Leg odometry update failed: {exc}', throttle_duration_sec=2.0)
            return
        self._last_ns = stamp_ns
        stamp = msg.header.stamp
        self._tf.sendTransform([
            _transform(est.odom_from_footprint, self.odom_frame, self.footprint_frame, stamp),
            _transform(est.footprint_from_base, self.footprint_frame, self.base_frame, stamp)])
        self._update_velocity(stamp_ns, est.odom_from_footprint)
        if self._odom_pub is not None:
            self._odom_pub.publish(self._odometry(est, stamp))

    def _update_velocity(self, stamp_ns, odom_fp):
        """Footprint velocity over >= 0.2 s (stance switches make single steps jumpy), low-passed."""
        if self._last_pose is None:
            self._last_pose = (stamp_ns, odom_fp)
            return
        t0, T0 = self._last_pose
        dt = (stamp_ns - t0) * 1e-9
        if dt < 0.2:
            return
        R = odom_fp[:3, :3]
        v = R.T @ (odom_fp[:3, 3] - T0[:3, 3]) / dt
        yaw = math.remainder(math.atan2(odom_fp[1, 0], odom_fp[0, 0]) - math.atan2(T0[1, 0], T0[0, 0]),
                             2 * math.pi)
        lin, ang = self._velocity
        self._velocity = (0.5 * lin + 0.5 * v, 0.5 * ang + 0.5 * yaw / dt)
        self._last_pose = (stamp_ns, odom_fp)

    def _odometry(self, est, stamp) -> Odometry:
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, self.odom_frame
        msg.child_frame_id = self.footprint_frame
        T = est.odom_from_footprint
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(T[0, 3]), float(T[1, 3])
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = matrix_to_quat_xyzw(T[:3, :3])
        lin, ang = self._velocity
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = float(lin[0]), float(lin[1])
        msg.twist.twist.angular.z = float(ang)
        # Dead reckoning: no absolute reference. Rough 1-sigma for a few seconds of walking.
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
        msg.pose.covariance[35] = math.radians(2.0) ** 2
        return msg

    def _publish_status(self):
        import json
        est = self.odometry
        self._status_pub.publish(String(data=json.dumps({
            'stance': est.stance, 'stance_switches': est.switches,
            'stance_source': 'walk_controller' if self._stance else 'lower_foot', 'skipped': self._skipped})))


def main(args=None):
    rclpy.init(args=args)
    node = LegOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
