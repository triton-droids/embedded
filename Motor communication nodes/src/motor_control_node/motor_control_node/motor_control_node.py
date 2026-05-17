import rclpy
from rclpy.node import Node


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.get_logger().info('Motor control node started')


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
