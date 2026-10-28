"""Start a simulated policy run: enable the motors and release the gantry.

The tracking policy plays its embedded reference motion (299 frames, ~6 s)
from the moment its gyro calibration finishes, whether or not the motors are
enabled. Done by hand, the clip is usually over before the robot is on the
floor. This node waits for the policy's first command on
/policy/desired_motor_subset (reference frame 0), then once:

  1. publishes ENABLE for the policy joints on /motor_commands, the same
     message an operator would send on hardware;
  2. after release_delay_s, calls /sim/hold_base false.

It is simulation-only and never re-arms: after /sim/reset, restart the policy
node and this node (or enable/release by hand).
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from motor_control_interfaces.msg import MotorCommand


class SimAutostartNode(Node):

    def __init__(self) -> None:
        super().__init__('sim_autostart')
        self.declare_parameter('release_delay_s', 0.1)
        self.declare_parameter('policy_topic', '/policy/desired_motor_subset')
        self._release_delay = float(self.get_parameter('release_delay_s').value)
        if not 0.0 <= self._release_delay < 0.5:
            # A bare ENABLE expires after the backend's 0.5 s command timeout.
            raise ValueError('release_delay_s must be in [0, 0.5)')
        self._command_pub = self.create_publisher(MotorCommand, '/motor_commands', 10)
        self._hold_client = self.create_client(SetBool, '/sim/hold_base')
        # Matches the policy's best-effort stream publisher.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._policy_sub = self.create_subscription(
            MotorCommand, str(self.get_parameter('policy_topic').value), self._on_policy, qos)
        self._release_timer = None
        self._done = False
        self.get_logger().info('Waiting for the first policy command to enable motors and '
                               'release the gantry')

    def _on_policy(self, msg: MotorCommand) -> None:
        if self._done or self._release_timer is not None:
            return
        enable = MotorCommand()
        enable.header.stamp = self.get_clock().now().to_msg()
        enable.joint_name = list(msg.joint_name)
        enable.mode = [MotorCommand.MODE_ENABLE]
        self._command_pub.publish(enable)
        self.get_logger().warn(f'Policy started; enabled {len(enable.joint_name)} motors')
        self.destroy_subscription(self._policy_sub)
        self._release_timer = self.create_timer(self._release_delay, self._release)

    def _release(self) -> None:
        self._release_timer.cancel()
        if not self._hold_client.service_is_ready():
            self.get_logger().error('/sim/hold_base is not available; gantry not released')
            self._done = True
            return
        future = self._hold_client.call_async(SetBool.Request(data=False))
        future.add_done_callback(self._released)

    def _released(self, future) -> None:
        self._done = True
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error('Gantry release failed')
            return
        self.get_logger().warn('Gantry released')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimAutostartNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
