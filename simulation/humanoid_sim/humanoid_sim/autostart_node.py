"""Start a simulated policy run: enable the motors and release the gantry.

The tracking policy plays its embedded reference motion (299 frames, ~6 s)
from the moment its gyro calibration finishes, whether or not the motors are
enabled. Done by hand, the clip is usually over before the robot is on the
floor. This node waits for a fresh command on /policy/desired_motor_subset
and discovery of the gantry service and motor subscriber, then once:

  1. publishes ENABLE for the policy joints on /motor_commands, the same
     message an operator would send on hardware;
  2. after release_delay_s, requires the continuing fresh command stream and
     asks the backend to verify accepted motion before releasing /sim/hold_base.

It is simulation-only and never re-arms: after /sim/reset, restart the policy
node and this node (or enable/release by hand).

With enable_via_control_core, ENABLE uses the existing C++ action service.
Gantry release additionally requires matching COMPLETED enabled feedback and
fresh C++ motor output carrying a recently observed upstream source stamp.
"""
from __future__ import annotations

from collections import deque
import math
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import SetBool

from motor_control_interfaces.msg import MotorCommand, AbstractCommand, AbstractCommandStatus
from motor_control_interfaces.srv import SubmitAbstractCommand


class SimAutostartNode(Node):

    def __init__(self) -> None:
        super().__init__('sim_autostart')
        self.declare_parameter('release_delay_s', 0.1)
        self.declare_parameter('startup_timeout_s', 2.0)
        self.declare_parameter('policy_topic', '/policy/desired_motor_subset')
        self.declare_parameter('enable_via_control_core', False)
        self._use_control_core = bool(self.get_parameter('enable_via_control_core').value)
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
        self._latest_policy_deadline_ns = 0
        self._last_release_denial = ''
        self._enabled_joints = []
        self._candidate_joints = []
        self._waiting_logged = False
        self._enable_id = 'sim_autostart_' + uuid.uuid4().hex
        self._enable_pending = False
        self._enable_accepted = False
        self._enable_request_at = None
        self._policy_sources = deque(maxlen=128)
        self._forwarded_deadline_ns = 0
        self._forwarded_ready_since = None
        self._last_forwarded_at = None
        if self._use_control_core:
            self._enable_client = self.create_client(
                SubmitAbstractCommand, '/control_core/submit_abstract_command')
            self.create_subscription(
                AbstractCommandStatus, '/control_core/abstract_command_status',
                self._on_enable_status, QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(MotorCommand, '/motor_commands', self._on_forwarded, 10)
        self.create_timer(0.05, self._check_timeout)
        self.get_logger().info('Waiting for the first policy command to enable motors and '
                               'release the gantry')

    def _on_policy(self, msg: MotorCommand) -> None:
        if self._done:
            return
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        deadline_ns = msg.valid_until.sec * 10**9 + msg.valid_until.nanosec
        # Discovery may deliver a queued command; never start on expired input.
        if (not msg.joint_name or stamp_ns <= 0 or stamp_ns > now_ns + 10_000_000
                or deadline_ns <= max(now_ns, stamp_ns)):
            return
        self._latest_policy_deadline_ns = deadline_ns
        if self._use_control_core:
            self._candidate_joints = list(msg.joint_name)
            self._policy_sources.append(stamp_ns)
        if self._release_timer is not None:
            return
        if self._first_policy_at is None:
            self._first_policy_at = time.monotonic()
        if time.monotonic() - self._first_policy_at > self._startup_timeout:
            self._fail('Timed out waiting for simulation readiness/fresh commands')
            return
        if (not self._hold_client.service_is_ready()
                or self._command_pub.get_subscription_count() == 0):
            if not self._waiting_logged:
                self.get_logger().warn('Waiting for /sim/hold_base and motor command discovery; '
                                       'motors remain disabled')
                self._waiting_logged = True
            return
        if self._use_control_core:
            self._start_verified(msg)
            return
        enable = MotorCommand()
        enable.header.stamp = self.get_clock().now().to_msg()
        enable.joint_name = list(msg.joint_name)
        enable.mode = [MotorCommand.MODE_ENABLE]
        self._command_pub.publish(enable)
        self._enabled_joints = list(enable.joint_name)
        self.get_logger().warn(f'Policy started; sent ENABLE for {len(enable.joint_name)} motors')
        self._release_timer = self.create_timer(max(0.001, self._release_delay), self._release)

    def _start_verified(self, msg):
        if self._enable_pending or self._enable_accepted or not self._enable_client.service_is_ready():
            return
        if self._forwarded_deadline_ns <= self.get_clock().now().nanoseconds + 40_000_000:
            return
        if self._forwarded_ready_since is None or time.monotonic() - self._forwarded_ready_since < .3:
            return
        request = SubmitAbstractCommand.Request()
        request.command.header.stamp = self.get_clock().now().to_msg()
        request.command.valid_until = (
            self.get_clock().now() + rclpy.duration.Duration(seconds=.3)).to_msg()
        request.command.request_id = self._enable_id
        request.command.joint_name = '*'
        request.command.operation = AbstractCommand.ENABLE
        self._enabled_joints = list(msg.joint_name)
        self._enable_pending = True
        self._enable_request_at = time.monotonic()
        try:
            self._enable_client.call_async(request).add_done_callback(self._enable_reply)
        except Exception as exc:
            self._fail(f'C++ enable submission failed: {exc}')

    def _enable_reply(self, future):
        if self._done:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f'C++ enable reply failed: {exc}')
            return
        if response is None:
            self._fail('Missing C++ enable reply')
        elif response.accepted:
            self._forwarded_deadline_ns = 0
            self._enable_accepted = True
            self.get_logger().warn('C++ accepted ENABLE; waiting for verified enabled feedback')
        else:
            self._enable_pending = False
            self._enabled_joints = []
            self._last_release_denial = 'C++ enable not admitted: ' + response.message

    def _on_enable_status(self, msg):
        if self._done or not self._enable_pending or msg.request_id != self._enable_id:
            return
        if msg.stage in (AbstractCommandStatus.FAILED, AbstractCommandStatus.CANCELLED,
                         AbstractCommandStatus.REJECTED):
            self._fail('C++ enable failed: ' + msg.message)
        elif msg.stage == AbstractCommandStatus.COMPLETED:
            self._enable_pending = False
            self._enable_accepted = True
            self.get_logger().warn('C++ verified all motors enabled; waiting for fresh WBC motion')
            self._release_timer = self.create_timer(max(.001, self._release_delay), self._release)

    def _on_forwarded(self, msg):
        if self._done or not msg.mode or any(mode != MotorCommand.MODE_MOTION for mode in msg.mode):
            return
        source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        deadline = msg.valid_until.sec * 10**9 + msg.valid_until.nanosec
        now = self.get_clock().now().nanoseconds
        if (deadline > now and source <= now + 10_000_000
                and set(msg.joint_name) == set(self._enabled_joints or getattr(self, '_candidate_joints', []))
                and any(abs(source - known) <= 1000 for known in self._policy_sources)):
            received = time.monotonic()
            if self._last_forwarded_at is None or received - self._last_forwarded_at > .05:
                self._forwarded_ready_since = received
            self._last_forwarded_at = received
            self._forwarded_deadline_ns = deadline

    def _fail(self, reason: str) -> None:
        self._done = True
        if self._enabled_joints:
            disable = MotorCommand()
            disable.header.stamp = self.get_clock().now().to_msg()
            disable.joint_name = self._enabled_joints
            disable.mode = [MotorCommand.MODE_DISABLE]
            self._command_pub.publish(disable)
        action = 'DISABLE sent' if self._enabled_joints else 'no ENABLE sent'
        denial = getattr(self, '_last_release_denial', '')
        self.get_logger().error(reason + (('; ' + denial) if denial else '') +
                                f'; autostart aborted, {action}')

    def _check_timeout(self) -> None:
        if self._done or self._first_policy_at is None:
            return
        if (self._request_at is None
                and time.monotonic() - self._first_policy_at > self._startup_timeout):
            self._fail('Timed out waiting for simulation readiness/fresh commands')
        elif (self._request_at is not None
                and time.monotonic() - self._request_at > self._startup_timeout):
            self._fail('Timed out waiting for gantry release response')

    def _release(self) -> None:
        if self._done:
            return
        if not self._hold_client.service_is_ready():
            self._fail('/sim/hold_base became unavailable; gantry not released')
            return
        if self._latest_policy_deadline_ns <= self.get_clock().now().nanoseconds + 40_000_000:
            return  # Keep the gantry while waiting for the continuing fresh stream.
        if (self._use_control_core and
                self._forwarded_deadline_ns <= self.get_clock().now().nanoseconds + 40_000_000):
            return
        self._release_timer.cancel()
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
        if response is not None and not response.success and getattr(response, 'message', '').startswith('Release denied:'):
            # The fresh policy stream may arrive before the backend command.
            # Retry readiness, never repeat ENABLE, within the startup budget.
            self._request_at = None
            self._last_release_denial = response.message
            self._release_timer.reset()
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
