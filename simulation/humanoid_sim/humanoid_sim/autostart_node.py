"""Start a simulated policy run: enable the motors and release the gantry.

The tracking policy plays its embedded reference motion (299 frames, ~6 s)
from the moment its gyro calibration finishes, whether or not the motors are
enabled. Done by hand, the clip is usually over before the robot is on the
floor. This node waits for a fresh command on /policy/desired_motor_subset
and discovery of the gantry service and motor subscriber, then once:

  1. publishes ENABLE for the policy joints on /motor_commands, the same
     message an operator would send on hardware;
  2. after release_delay_s, calls /sim/hold_base false.

It is simulation-only and never re-arms: after /sim/reset, restart the policy
node and this node (or enable/release by hand).
"""
from __future__ import annotations

import math
import time

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
        self.declare_parameter('startup_timeout_s', 2.0)
        self.declare_parameter('policy_topic', '/policy/desired_motor_subset')
        self._release_delay = float(self.get_parameter('release_delay_s').value)
        self._startup_timeout = float(self.get_parameter('startup_timeout_s').value)
        if not math.isfinite(self._startup_timeout) or self._startup_timeout <= 0:
            raise ValueError('startup_timeout_s must be finite and positive')
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
        self._first_policy_at = None
        self._request_at = None
        self._enabled_joints = []
        self._waiting_logged = False
        self.create_timer(0.05, self._check_timeout)
        self.get_logger().info('Waiting for the first policy command to enable motors and '
                               'release the gantry')

    def _on_policy(self, msg: MotorCommand) -> None:
        if self._done or self._release_timer is not None:
            return
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        deadline_ns = msg.valid_until.sec * 10**9 + msg.valid_until.nanosec
        # Discovery may deliver a queued command; never start on expired input.
        if (not msg.joint_name or stamp_ns <= 0 or stamp_ns > now_ns + 10_000_000
                or deadline_ns <= max(now_ns, stamp_ns)):
            return
        if self._first_policy_at is None:
            self._first_policy_at = time.monotonic()
        if time.monotonic() - self._first_policy_at > self._startup_timeout:
            self._fail('Timed out waiting for simulation service/command discovery')
            return
        if (not self._hold_client.service_is_ready()
                or self._command_pub.get_subscription_count() == 0):
            if not self._waiting_logged:
                self.get_logger().warn('Waiting for /sim/hold_base and motor command discovery; '
                                       'motors remain disabled')
                self._waiting_logged = True
            return
        enable = MotorCommand()
        enable.header.stamp = self.get_clock().now().to_msg()
        enable.joint_name = list(msg.joint_name)
        enable.mode = [MotorCommand.MODE_ENABLE]
        self._command_pub.publish(enable)
        self._enabled_joints = list(enable.joint_name)
        self.get_logger().warn(f'Policy started; sent ENABLE for {len(enable.joint_name)} motors')
        self.destroy_subscription(self._policy_sub)
        self._release_timer = self.create_timer(max(0.001, self._release_delay), self._release)

    def _fail(self, reason: str) -> None:
        self._done = True
        if self._enabled_joints:
            disable = MotorCommand()
            disable.header.stamp = self.get_clock().now().to_msg()
            disable.joint_name = self._enabled_joints
            disable.mode = [MotorCommand.MODE_DISABLE]
            self._command_pub.publish(disable)
        action = 'DISABLE sent' if self._enabled_joints else 'no ENABLE sent'
        self.get_logger().error(reason + f'; autostart aborted, {action}')

    def _check_timeout(self) -> None:
        if self._done or self._first_policy_at is None:
            return
        if (self._release_timer is None
                and time.monotonic() - self._first_policy_at > self._startup_timeout):
            self._fail('Timed out waiting for simulation service/command discovery')
        elif (self._request_at is not None
                and time.monotonic() - self._request_at > self._startup_timeout):
            self._fail('Timed out waiting for gantry release response')

    def _release(self) -> None:
        self._release_timer.cancel()
        if self._done:
            return
        if not self._hold_client.service_is_ready():
            self._fail('/sim/hold_base became unavailable; gantry not released')
            return
        try:
            future = self._hold_client.call_async(SetBool.Request(data=False))
        except Exception as exc:
            self._fail(f'Gantry release request failed: {exc}')
            return
        self._request_at = time.monotonic()
        future.add_done_callback(self._released)

    def _released(self, future) -> None:
        if self._done:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f'Gantry release failed: {exc}')
            return
        if response is None or not response.success:
            self._fail('Gantry release failed')
            return
        self._done = True
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
