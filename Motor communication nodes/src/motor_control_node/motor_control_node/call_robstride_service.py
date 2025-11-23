#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from motor_control_interfaces.srv import RobStrideJointControl


class RobStrideClient(Node):
    def __init__(self):
        super().__init__('robstride_example_client')
        self.cli = self.create_client(RobStrideJointControl, 'robstride_joint_control')

        self.get_logger().info('Waiting for service /robstride_joint_control...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        self.get_logger().info('Service is available.')

    def send_request(self, joint_name: str, command_type: int,
                     position: float = 0.0,
                     velocity: float = 0.0,
                     acceleration: float = 0.0,
                     torque: float = 0.0,
                     iq: float = 0.0,
                     id_: float = 0.0,
                     kp: float = 0.0,
                     kd: float = 0.0):
        req = RobStrideJointControl.Request()
        req.joint_name = joint_name
        req.command_type = command_type
        req.position = position
        req.velocity = velocity
        req.acceleration = acceleration
        req.torque = torque
        req.iq = iq
        req.id = id_
        req.kp = kp
        req.kd = kd

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            self.get_logger().info(
                f"Response: success={res.success}, message='{res.message}', "
                f"pos={res.position:.3f}, vel={res.velocity:.3f}, "
                f"torque={res.torque:.3f}, temp={res.temperature:.1f}"
            )
            return res
        else:
            self.get_logger().error('Service call failed!')
            raise RuntimeError('Service call failed')


def main(args=None):
    rclpy.init(args=args)
    node = RobStrideClient()

    joint = 'shoulder_pitch'

    # 1) ENABLE
    node.get_logger().info('--- Enabling motor ---')
    node.send_request(
        joint_name=joint,
        command_type=0,   # CMD_ENABLE
    )

    # 2) Move to ~45 degrees (0.785 rad) with 1 rad/s
    node.get_logger().info('--- Move to 0.785 rad (~45 deg) ---')
    node.send_request(
        joint_name=joint,
        command_type=3,   # CMD_POSITION_PP
        position=0.785,
        velocity=1.0,
        acceleration=1.0,
    )

    # 3) Move back to 0 rad
    node.get_logger().info('--- Move back to 0.0 rad ---')
    node.send_request(
        joint_name=joint,
        command_type=3,   # CMD_POSITION_PP
        position=0.0,
        velocity=1.0,
        acceleration=1.0,
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
