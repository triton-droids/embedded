#!/usr/bin/env python3
"""Expose a selected JointState stream for RViz over ROS 2 DDS."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def valid_joint_state(msg):
    """Positions are required; velocity and effort may be unavailable."""
    count = len(msg.name)
    return (
        count > 0
        and all(msg.name)
        and len(set(msg.name)) == count
        and len(msg.position) == count
        and all(len(values) in (0, count) for values in (msg.velocity, msg.effort))
        and all(math.isfinite(value)
                for values in (msg.position, msg.velocity, msg.effort) for value in values)
    )


class JointStateMonitorNode(Node):
    def __init__(self):
        super().__init__('joint_state_monitor_node')
        self.declare_parameter('source_topic', '/policy/joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        source = self.get_parameter('source_topic').value
        output = self.get_parameter('output_topic').value
        if self.resolve_topic_name(source) == self.resolve_topic_name(output):
            raise ValueError('source_topic and output_topic must differ')
        self.publisher = self.create_publisher(JointState, output, 5)
        self.subscription = self.create_subscription(
            JointState, source, self.on_joint_state, qos_profile_sensor_data)
        self.get_logger().info(f'Joint visualization: {source} -> {output}; preserving source timestamps')
        if source == '/policy/target_angles':
            self.get_logger().warn('Displaying commanded policy targets, not measured joint positions')

    def on_joint_state(self, msg):
        if not valid_joint_state(msg):
            self.get_logger().warn('Dropped invalid JointState', throttle_duration_sec=5.0)
            return
        # Event-driven forwarding: never fabricate velocities or repeat stale poses.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitorNode()
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
