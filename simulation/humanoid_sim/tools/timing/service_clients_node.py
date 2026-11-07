#!/usr/bin/env python3
"""A node that only creates the two simulator service clients cpp_control_node uses.

It never sends a request. With Fast DDS, its joining the graph is enough to
make a simulator publish() block ~330 ms about 2 s later (dds_join_repro.sh).
"""
import rclpy

from motor_control_interfaces.srv import ExecuteMotorOperation, GetMotorFeedback


def main():
    rclpy.init()
    node = rclpy.create_node('timing_service_clients')
    node.create_client(GetMotorFeedback, '/motor_backend/get_feedback')
    node.create_client(ExecuteMotorOperation, '/motor_backend/execute_operation')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
