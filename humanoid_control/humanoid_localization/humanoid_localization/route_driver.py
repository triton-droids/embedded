"""route_driver: lead the ball-chasing robot around a long route (simulation test only).

  ros2 run humanoid_localization route_driver --ros-args -p legs:=16

Whenever the robot has reached the ball (ground-truth distance below
``reached_m``), the ball is placed ``lead_m`` ahead of the robot, turned
``turn_deg`` to the left of its heading, so the robot walks a polygon and keeps
turning. The ball stays inside the head camera's field of view, so the normal
vision chase follows it. This node plays the referee moving the ball: it reads
simulator ground truth, which the localizer never does. It exits after ``legs``
placements.

Until ``/localization/status`` reports ``initialized`` the ball is parked
behind the robot (out of view), so the robot stands still at its known start
pose while visual odometry starts up.
"""
from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .fusion import quat_to_matrix


class RouteDriver(Node):
    def __init__(self):
        super().__init__('route_driver')
        defaults = {'legs': 16, 'lead_m': 1.3, 'turn_deg': 35.0, 'reached_m': 0.7,
                    'settle_s': 1.0, 'leg_timeout_s': 40.0, 'ball_z': 0.11,
                    'start_delay_s': 5.0, 'wait_localization': True,
                    'park_x': 0.0, 'park_y': -3.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.ball_pub = self.create_publisher(PointStamped, '/ball_chase/throw_position', 2)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/ball_chase/ground_truth/ball', self._on_ball, 2)
        self.create_subscription(String, '/localization/status', self._on_status, 2)
        self.create_timer(0.2, self._step)
        self.localized = not bool(self.p['wait_localization'])
        self.localized_at = None
        self.pose = None
        self.ball = None
        self.leg = 0
        self.leg_start = None
        self.reached_since = None
        self.started = self.get_clock().now()
        self.done = False

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (np.array([p.x, p.y]), quat_to_matrix((q.w, q.x, q.y, q.z)))

    def _on_ball(self, msg):
        self.ball = np.array([msg.point.x, msg.point.y])

    def _on_status(self, msg):
        try:
            initialized = bool(json.loads(msg.data).get('initialized'))
        except (ValueError, AttributeError):
            return
        if initialized and not self.localized:
            self.localized = True
            self.localized_at = self.get_clock().now()
            self.get_logger().warn('Localization initialized; starting the route')

    def _place(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(self.p['ball_z'])
        self.ball_pub.publish(msg)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _place_next(self):
        xy, rot = self.pose
        forward = rot[:2, 1]  # Base forward axis is +y (sim_camera base_forward_axis).
        forward = forward / np.linalg.norm(forward)
        a = math.radians(float(self.p['turn_deg']))
        direction = np.array([math.cos(a) * forward[0] - math.sin(a) * forward[1],
                              math.sin(a) * forward[0] + math.cos(a) * forward[1]])
        target = xy + float(self.p['lead_m']) * direction
        self._place(*target)
        self.leg += 1
        self.leg_start = self._now_s()
        self.reached_since = None
        self.get_logger().warn(f'Leg {self.leg}/{self.p["legs"]}: ball at ({target[0]:.2f}, {target[1]:.2f}), '
                               f'robot at ({xy[0]:.2f}, {xy[1]:.2f})')

    def _step(self):
        if self.done or self.pose is None or self.ball is None:
            return
        if not self.localized:
            if np.linalg.norm(self.ball - (self.p['park_x'], self.p['park_y'])) > 0.2:
                self._place(self.p['park_x'], self.p['park_y'])  # Out of view: the robot stands.
            return
        since = self.localized_at or self.started
        if (self.get_clock().now() - since).nanoseconds * 1e-9 < float(self.p['start_delay_s']):
            return
        now = self._now_s()
        distance = float(np.linalg.norm(self.ball - self.pose[0]))
        if distance < float(self.p['reached_m']):
            self.reached_since = self.reached_since or now
        else:
            self.reached_since = None
        reached = self.reached_since is not None and now - self.reached_since >= float(self.p['settle_s'])
        timed_out = self.leg_start is not None and now - self.leg_start > float(self.p['leg_timeout_s'])
        if self.leg_start is None or reached or timed_out:
            if timed_out and not reached:
                self.get_logger().warn(f'Leg {self.leg} timed out at distance {distance:.2f} m')
            if self.leg >= int(self.p['legs']):
                self.get_logger().warn('Route finished')
                self.done = True
                return
            self._place_next()


def main(args=None):
    rclpy.init(args=args)
    node = RouteDriver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
