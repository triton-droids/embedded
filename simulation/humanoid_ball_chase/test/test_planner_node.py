"""DDS round trip for map/goal/odometry -> Path and invalidation."""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from humanoid_ball_chase.planner_node import AStarPlanner


def test_ros_path_round_trip_and_invalidations():
    rclpy.init()
    planner, peer = AStarPlanner(), Node('astar_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(peer)
    paths, statuses = [], []
    peer.create_subscription(Path, '/ball_chase/global_path', paths.append, 1)
    peer.create_subscription(String, '/ball_chase/planner_status',
                             lambda m: statuses.append(m.data), 1)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = peer.create_publisher(OccupancyGrid, '/ball_chase/map', qos)
    goal_pub = peer.create_publisher(PoseStamped, '/ball_chase/goal', 1)
    odom_pub = peer.create_publisher(Odometry, '/sim/ground_truth/odom', 1)
    grid = OccupancyGrid()
    grid.header.frame_id = 'world'
    grid.info.width = grid.info.height = 20
    grid.info.resolution = 0.2
    grid.info.origin.orientation.w = 1.0
    grid.data = [0]*400
    # Wall with room to detour above it.
    for y in range(14):
        grid.data[y*20+10] = 100
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x, goal.pose.position.y = 3.3, 1.1
    goal.pose.orientation.w = 1.0
    odom = Odometry()
    odom.header.frame_id = 'world'
    odom.pose.pose.position.x, odom.pose.pose.position.y = 0.9, 1.1

    def wait_for(status, refresh=True):
        statuses.clear()
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if refresh:
                odom.header.stamp = peer.get_clock().now().to_msg()
                odom_pub.publish(odom)
            map_pub.publish(grid)
            goal_pub.publish(goal)
            executor.spin_once(timeout_sec=0.02)
            if status in statuses and paths:
                return
        raise AssertionError((status, statuses))

    try:
        wait_for('planned')
        assert paths[-1].header.frame_id == 'world'
        assert max(p.pose.position.y for p in paths[-1].poses) >= 3.1
        goal.header.frame_id = 'other'
        wait_for('frame_mismatch')
        assert not paths[-1].poses
        goal.header.frame_id = 'world'
        grid.info.origin.orientation.z = 0.5
        wait_for('rotated_or_invalid_map_origin')
        assert not paths[-1].poses
        grid.info.origin.orientation.z = 0.0
        odom.header.stamp.sec -= 10
        odom_pub.publish(odom)
        wait_for('stale_pose', refresh=False)
        assert not paths[-1].poses
    finally:
        executor.shutdown()
        planner.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
