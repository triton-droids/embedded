"""vio_localizer: base pose from the body IMU and stereo visual odometry.

Subscribes
  /imu/data_raw                     sensor_msgs/Imu (BNO085 game rotation vector)
  /visual_slam/tracking/odometry    nav_msgs/Odometry, pose of ``rig_frame`` (cuVSLAM)
  TF ``base_frame`` -> ``rig_frame`` at each VO stamp (head forward kinematics)

Publishes
  /localization/odom     nav_msgs/Odometry, frame ``world``, child ``base_frame``;
                         twist is world-frame like /sim/ground_truth/odom
  /localization/status   std_msgs/String JSON, 1 Hz

The start pose (``initial_x``, ``initial_y``, ``initial_z``, ``initial_yaw``)
is the known placement, e.g. the kickoff position; it is not read from the
simulator. Nothing here subscribes to simulator ground truth.
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .fusion import FusionConfig, VioFusion, matrix_to_quat, quat_to_matrix, transform


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _pose_matrix(pose) -> np.ndarray:
    q = pose.orientation
    p = pose.position
    return transform(quat_to_matrix((q.w, q.x, q.y, q.z)), (p.x, p.y, p.z))


class VioLocalizer(Node):
    def __init__(self):
        super().__init__('vio_localizer')
        defaults = {
            'imu_topic': '/imu/data_raw',
            'vo_topic': '/visual_slam/tracking/odometry',
            'odom_topic': '/localization/odom',
            'world_frame': 'world',
            'base_frame': 'floating_base',
            'rig_frame': 'stereo_rig',
            'initial_x': 0.0, 'initial_y': 0.0, 'initial_z': 0.0, 'initial_yaw': 0.0,
            'max_speed_m_s': 2.0,
            'publish_rate_hz': 50.0,
            'tf_timeout_s': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.p = p
        self.fusion = VioFusion(FusionConfig(
            initial_position=(float(p['initial_x']), float(p['initial_y']), float(p['initial_z'])),
            initial_yaw=float(p['initial_yaw']), max_speed_m_s=float(p['max_speed_m_s'])))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, p['odom_topic'], 2)
        self.status_pub = self.create_publisher(String, '/localization/status', 2)
        self.create_subscription(Imu, p['imu_topic'], self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['vo_topic'], self._on_vo, qos_profile_sensor_data)
        self.create_timer(1.0 / float(p['publish_rate_hz']), self._publish)
        self.create_timer(1.0, self._publish_status)
        self.outcomes = {}
        self.tf_failures = 0
        self.last_vo_stamp = None
        self.last_imu_stamp = None
        self.velocity = np.zeros(3)
        self._velocity_sample = None

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        if msg.orientation_covariance[0] < 0:
            return  # No orientation in this message.
        try:
            self.fusion.add_imu(_stamp_s(msg.header.stamp), (q.w, q.x, q.y, q.z))
        except ValueError:
            return
        self.last_imu_stamp = msg.header.stamp

    def _on_vo(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.p['base_frame'], self.p['rig_frame'], stamp,
                timeout=Duration(seconds=float(self.p['tf_timeout_s'])))
        except TransformException:
            self.tf_failures += 1
            return
        r, t = tf.transform.rotation, tf.transform.translation
        base_from_rig = transform(quat_to_matrix((r.w, r.x, r.y, r.z)), (t.x, t.y, t.z))
        stamp_s = _stamp_s(msg.header.stamp)
        outcome = self.fusion.add_vo(stamp_s, _pose_matrix(msg.pose.pose), base_from_rig)
        self.outcomes[outcome] = self.outcomes.get(outcome, 0) + 1
        self.last_vo_stamp = msg.header.stamp
        if outcome == 'accepted':
            if self._velocity_sample is not None and stamp_s > self._velocity_sample[0]:
                self.velocity = ((self.fusion.position - self._velocity_sample[1])
                                 / (stamp_s - self._velocity_sample[0]))
            self._velocity_sample = (stamp_s, self.fusion.position.copy())
        elif outcome in ('restarted', 'initialized'):
            self._velocity_sample = (stamp_s, self.fusion.position.copy())
            self.velocity = np.zeros(3)

    def _publish(self):
        orientation = self.fusion.orientation()
        if orientation is None or not self.fusion.initialized or self.last_imu_stamp is None:
            return
        msg = Odometry()
        msg.header.stamp = self.last_imu_stamp
        msg.header.frame_id = self.p['world_frame']
        msg.child_frame_id = self.p['base_frame']
        pose = msg.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, self.fusion.position)
        w, x, y, z = matrix_to_quat(orientation)
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
            float(w), float(x), float(y), float(z))
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(
            float, self.velocity)
        self.odom_pub.publish(msg)

    def _publish_status(self):
        now = self.get_clock().now()

        def age(stamp):
            return None if stamp is None else round((now - Time.from_msg(stamp)).nanoseconds * 1e-9, 3)
        status = {
            'initialized': self.fusion.initialized,
            'vo_outcomes': self.outcomes,
            'tf_failures': self.tf_failures,
            'vo_age_s': age(self.last_vo_stamp),
            'imu_age_s': age(self.last_imu_stamp),
            'position': None if self.fusion.position is None else [round(float(v), 3) for v in self.fusion.position],
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = VioLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
