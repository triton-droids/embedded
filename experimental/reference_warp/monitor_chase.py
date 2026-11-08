"""Record a running combined experiment; optional simulator ball occlusion."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, TwistStamped
from motor_control_interfaces.msg import MotorCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--occlude-at', type=float)
    parser.add_argument('--near-at', type=float, help='Place ball at world (0, 1.2, 0.11)')
    parser.add_argument('--far-at', type=float, help='Restore ball at world (0, 1.8, 0.11)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('combined_chase_monitor')
    begin = time.monotonic()
    data = {}

    def save(topic, value):
        data.setdefault(topic, []).append(dict(t=time.monotonic()-begin, value=value))

    for topic in ('/experimental/chase_status', '/ball_chase/status', '/policy/status', '/sim/status', '/wbc/status'):
        node.create_subscription(String, topic, lambda m, t=topic: save(t, json.loads(m.data)),
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))

    def motor(topic, msg):
        save(topic, dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                         valid_until_ns=msg.valid_until.sec*10**9+msg.valid_until.nanosec,
                         names=list(msg.joint_name), modes=list(msg.mode)))

    for topic in ('/policy/desired_motor_subset', '/desired_motor_subset', '/motor_commands'):
        node.create_subscription(MotorCommand, topic, lambda m, t=topic: motor(t, m),
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(TwistStamped, '/ball_chase/cmd_vel',
                             lambda m: save('cmd', dict(vx=m.twist.linear.x, wz=m.twist.angular.z)), 1)
    node.create_subscription(Odometry, '/sim/ground_truth/odom',
                             lambda m: save('odom', dict(position=[m.pose.pose.position.x,
                                m.pose.pose.position.y, m.pose.pose.position.z])), QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(DiagnosticArray, '/safety/status', lambda m: save('safety', [
        dict(name=s.name, message=s.message, values={v.key:v.value for v in s.values})
        for s in m.status]), QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                             lambda m: save('depth', dict(encoding=m.encoding, width=m.width,
                                 height=m.height, frame=m.header.frame_id)), QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    ball = node.create_publisher(PointStamped, '/ball_chase/throw_position', 1)
    events = [(args.near_at, 1.2, 'ball_placed_near'),
              (args.far_at, 1.8, 'ball_placed_far'),
              (args.occlude_at, -2.0, 'ball_placed_behind_robot')]
    sent = set()
    try:
        while time.monotonic()-begin < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            for at, y, label in events:
                if at is not None and label not in sent and time.monotonic()-begin >= at:
                    msg = PointStamped()
                    msg.header.frame_id = 'world'
                    msg.point.y, msg.point.z = y, 0.11
                    ball.publish(msg)
                    save('event', label)
                    sent.add(label)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2)+'\n')
        print(json.dumps(dict(output=str(args.output), counts={k:len(v) for k,v in data.items()})))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
