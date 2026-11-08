"""Optional simulation-only A* path publisher. Never commands motors or velocity."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from .astar import plan


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('ball_chase_astar')
        self.declare_parameter('robot_radius_m', 0.25)
        self.declare_parameter('pose_timeout_s', 1.0)
        self.radius = float(self.get_parameter('robot_radius_m').value)
        self.timeout = float(self.get_parameter('pose_timeout_s').value)
        if not math.isfinite(self.radius) or self.radius < 0 or not 0 < self.timeout <= 5:
            raise ValueError('Invalid radius or pose timeout')
        self.grid = self.goal = self.odom = None
        self.received = 0.0
        self.path_pub = self.create_publisher(Path, '/ball_chase/global_path', 2)
        self.status_pub = self.create_publisher(String, '/ball_chase/planner_status', 2)
        map_qos = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/ball_chase/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/ball_chase/goal', self.on_goal, 2)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_odom, 2)
        self.create_timer(0.5, self.replan)

    def on_map(self, msg):
        self.grid = msg
        self.replan()

    def on_goal(self, msg):
        self.goal = msg
        self.replan()

    def on_odom(self, msg):
        self.odom, self.received = msg, time.monotonic()

    def replan(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.grid.header.frame_id if self.grid else 'world'
        status = 'waiting_for_map_goal_pose'
        if self.grid is not None and self.goal is not None and self.odom is not None:
            grid, goal, odom = self.grid, self.goal, self.odom
            info = grid.info
            q = info.origin.orientation
            age = (self.get_clock().now().nanoseconds
                   - (odom.header.stamp.sec*10**9+odom.header.stamp.nanosec))/1e9
            if (time.monotonic()-self.received > self.timeout
                    or not 0 <= age <= self.timeout):
                status = 'stale_pose'
            elif (not grid.header.frame_id or goal.header.frame_id != grid.header.frame_id
                  or odom.header.frame_id != grid.header.frame_id):
                status = 'frame_mismatch'
            elif not (abs(q.x) < 1e-8 and abs(q.y) < 1e-8 and abs(q.z) < 1e-8
                      and abs(abs(q.w)-1) < 1e-8):
                status = 'rotated_or_invalid_map_origin'
            else:
                try:
                    if info.width < 1 or info.height < 1 or info.width*info.height > 250000:
                        raise ValueError('Invalid grid dimensions')
                    cells = np.asarray(grid.data).reshape(info.height, info.width)
                    start, end = odom.pose.pose.position, goal.pose.position
                    route = plan(cells, info.resolution,
                                 (info.origin.position.x, info.origin.position.y),
                                 (start.x, start.y), (end.x, end.y), self.radius)
                    for x, y in route:
                        pose = PoseStamped()
                        pose.header = path.header
                        pose.pose.position.x, pose.pose.position.y = x, y
                        pose.pose.orientation.w = 1.0
                        path.poses.append(pose)
                    status = 'planned' if route else 'blocked_unreachable_or_budget'
                except (ValueError, TypeError, OverflowError):
                    status = 'invalid_map_or_coordinates'
        # Publish empty paths on every invalid/lost-input condition to clear old routes.
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
