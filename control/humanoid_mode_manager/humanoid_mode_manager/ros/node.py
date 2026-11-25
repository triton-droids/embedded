"""ROS wrapper of the mode machine (machine.py). Placeholder: dry_run by default.

Inputs
  /safety/estop                         std_msgs/Bool (reliable, transient local)
  /safety/status                        diagnostic_msgs/DiagnosticArray, 'humanoid_safety/supervisor'
  /joint_states                         sensor_msgs/JointState (freshness only)
  /imu/data_raw                         sensor_msgs/Imu (tilt, |gyro|)
  controller_status_topic               std_msgs/String JSON (/walk/status or /policy/status)
  /sim/status                           std_msgs/String JSON 'base_held' (simulation gantry)
  /safety/fall_warning                  std_msgs/Bool (learned fall predictor; optional)
  cmd_vel_in_topic                      geometry_msgs/TwistStamped (task layer, e.g. /ball_chase/cmd_vel)
  /control_core/abstract_command_status motor_control_interfaces/AbstractCommandStatus
  /localization/status                  std_msgs/String JSON (VIO health -> nav sub-mode)
  /mode_manager/request                 std_msgs/String: enable, disable, reset, release,
                                        kick_left, kick_right, squat
Services
  /mode_manager/{enable,disable,reset,release}   std_srvs/Trigger (same as the requests)
Outputs
  /mode_manager/state                   std_msgs/String JSON (latched): mode, reason, history, ...
  cmd_vel_out_topic                     TwistStamped, the task velocity passed on only while
                                        motion is allowed (gate_cmd_vel); else a zero command
  /walk/kick                            std_msgs/String
  /control_core/submit_abstract_command ENABLE / DISABLE / HOME
  /safety/reset                         std_srvs/Trigger
  /policy/switch_model                  SwitchPolicyModel (only where the interface is built)

With dry_run (the default) the node computes and publishes the mode but executes no action
that reaches the motors or other nodes' state: every action is logged and listed in the
state JSON. Actions whose ROS interface does not exist yet (source selection, damping,
controller and localization reset) are logged as placeholders even with dry_run false.
"""
from __future__ import annotations

from collections import deque
import json
import math
import uuid

import rclpy
from rclpy.duration import Duration
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from motor_control_interfaces.msg import AbstractCommand, AbstractCommandStatus
from motor_control_interfaces.srv import SubmitAbstractCommand

try:  # SwitchPolicyModel; absent where motor_control_interfaces predates the model switch.
    from motor_control_interfaces.srv import SwitchPolicyModel
except ImportError:  # pragma: no cover - depends on the built interfaces
    SwitchPolicyModel = None

from ..core.actions import (Damp, Kick, ResetController, ResetLocalization, ResetSafety, SelectSource,
                      SubmitCommand, SwitchModel)
from ..core.machine import MachineParams, ModeMachine
from ..core.observation import (Observation, parse_base_held, parse_controller_status,
                          parse_localization_status, tilt_from_quaternion)

OPERATIONS = {'enable': AbstractCommand.ENABLE, 'disable': AbstractCommand.DISABLE,
              'home': AbstractCommand.HOME, 'cancel': AbstractCommand.CANCEL,
              'reset': AbstractCommand.RESET}
STAGES = {AbstractCommandStatus.IDLE: 'idle', AbstractCommandStatus.DECELERATING: 'decelerating',
          AbstractCommandStatus.SETTLING: 'settling', AbstractCommandStatus.WAITING_BACKEND: 'waiting_backend',
          AbstractCommandStatus.WAITING_REBIND: 'waiting_rebind', AbstractCommandStatus.RESUMING: 'resuming',
          AbstractCommandStatus.COMPLETED: 'completed', AbstractCommandStatus.FAILED: 'failed',
          AbstractCommandStatus.CANCELLED: 'cancelled', AbstractCommandStatus.REJECTED: 'rejected'}
SUPERVISOR = 'humanoid_safety/supervisor'
LATCHED = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)


def _level(status: DiagnosticStatus) -> int:
    level = status.level
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


class ModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mode_manager')
        defaults = MachineParams()
        machine_params = {name: getattr(defaults, name) for name in vars(defaults)}
        for name, value in machine_params.items():
            if name != 'external_enable':
                self.declare_parameter(name, value)
        node_params = {
            'dry_run': True,
            'rate_hz': 50.0,
            'controller_status_topic': '',      # Empty: /walk/status or /policy/status by controller.
            'joint_timeout_s': 0.2,
            'imu_timeout_s': 0.2,
            'status_timeout_s': 3.0,            # /walk/status is 2 Hz, /policy/status 1 Hz.
            'cmd_vel_timeout_s': 0.5,
            'cmd_vel_deadband': 1e-3,
            'cmd_vel_in_topic': '/ball_chase/cmd_vel',
            'cmd_vel_out_topic': '/mode_manager/cmd_vel',
            'gate_cmd_vel': False,
            'walk_reset_service': '/walk/reset',              # Placeholder: not implemented yet.
            'localization_reset_service': '/localization/reset',  # Placeholder: not implemented yet.
        }
        for name, value in node_params.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        # external_enable: default true in dry run, where this node enables nothing itself.
        self.declare_parameter('external_enable', bool(get('dry_run')))
        self.machine = ModeMachine(MachineParams(**{n: get(n) for n in machine_params}),
                                   request_prefix='mode_manager_' + uuid.uuid4().hex[:8])
        self.dry_run = bool(get('dry_run'))
        self.cfg = {n: get(n) for n in node_params}
        status_topic = self.cfg['controller_status_topic'] or (
            '/walk/status' if self.machine.p.controller == 'walk' else '/policy/status')

        # Latest inputs: value plus receipt time on the node clock (simulated time with use_sim_time).
        self.estop = None
        self.supervisor = (None, '', False)
        self.joints_at = self.imu_at = self.status_at = self.cmd_vel_at = None
        self.tilt = self.angular_speed = None
        self.controller = None
        self.base_held = None
        self.fall_warning = False
        self.cmd_vel = None
        self.command_stages = {}
        self.localization = None
        self.executed = deque(maxlen=30)

        self.create_subscription(Bool, '/safety/estop', self._on_estop, LATCHED)
        self.create_subscription(DiagnosticArray, '/safety/status', self._on_supervisor, BEST_EFFORT)
        self.create_subscription(JointState, '/joint_states', self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(String, status_topic, self._on_controller, 10)
        self.create_subscription(String, '/sim/status', self._on_sim, 10)
        self.create_subscription(Bool, '/safety/fall_warning', self._on_fall_warning, LATCHED)
        self.create_subscription(TwistStamped, self.cfg['cmd_vel_in_topic'], self._on_cmd_vel, BEST_EFFORT)
        self.create_subscription(AbstractCommandStatus, '/control_core/abstract_command_status',
                                 self._on_command_status, LATCHED)
        self.create_subscription(String, '/localization/status', self._on_localization, 10)
        self.create_subscription(String, '/mode_manager/request', self._on_request, 10)
        for name in ('enable', 'disable', 'reset', 'release'):
            self.create_service(Trigger, f'/mode_manager/{name}',
                                lambda req, res, name=name: self._on_trigger(name, res))

        self.state_pub = self.create_publisher(String, '/mode_manager/state', LATCHED)
        self.kick_pub = self.create_publisher(String, '/walk/kick', 10)
        self.cmd_vel_pub = (self.create_publisher(TwistStamped, self.cfg['cmd_vel_out_topic'], BEST_EFFORT)
                            if self.cfg['gate_cmd_vel'] else None)
        self.command_client = self.create_client(SubmitAbstractCommand, '/control_core/submit_abstract_command')
        self.safety_reset_client = self.create_client(Trigger, '/safety/reset')
        self.walk_reset_client = self.create_client(Trigger, self.cfg['walk_reset_service'])
        self.localization_reset_client = self.create_client(Trigger, self.cfg['localization_reset_service'])
        self.switch_client = (self.create_client(SwitchPolicyModel, '/policy/switch_model')
                              if SwitchPolicyModel is not None else None)

        self.create_timer(1.0 / float(self.cfg['rate_hz']), self._tick)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().warn(
            f'Mode manager ({"DRY RUN: no actions executed" if self.dry_run else "LIVE"}), '
            f'controller {self.machine.p.controller}, status {status_topic}')

    # ------------------------------------------------------------------ inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    def _on_supervisor(self, msg):
        for status in msg.status:
            if status.name == SUPERVISOR:
                # ERROR: latched (the estop follows); WARN: a check initializing or unhealthy.
                level = _level(status)
                self.supervisor = (level != DiagnosticStatus.ERROR, status.message,
                                   level == DiagnosticStatus.WARN)

    def _on_joints(self, msg):
        self.joints_at = self._now()

    def _on_imu(self, msg):
        q, w = msg.orientation, msg.angular_velocity
        if msg.orientation_covariance[0] >= 0:
            self.tilt = tilt_from_quaternion(q.x, q.y, q.z, q.w)
        speed = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        self.angular_speed = speed if math.isfinite(speed) else None
        self.imu_at = self._now()

    def _on_controller(self, msg):
        status = parse_controller_status(msg.data)
        if status is not None:
            self.controller, self.status_at = status, self._now()

    def _on_sim(self, msg):
        held = parse_base_held(msg.data)
        if held is not None:
            self.base_held = held

    def _on_fall_warning(self, msg):
        self.fall_warning = bool(msg.data)

    def _on_cmd_vel(self, msg):
        self.cmd_vel, self.cmd_vel_at = msg, self._now()

    def _on_command_status(self, msg):
        if msg.request_id:
            self.command_stages[msg.request_id] = STAGES.get(msg.stage, str(msg.stage))
            while len(self.command_stages) > 32:
                self.command_stages.pop(next(iter(self.command_stages)))

    def _on_localization(self, msg):
        self.localization = parse_localization_status(msg.data)

    def _on_request(self, msg):
        if not self.machine.request(msg.data.strip()):
            self.get_logger().warn(f'Unknown request {msg.data!r}')

    def _on_trigger(self, name, response):
        response.success = self.machine.request(name)
        response.message = f'{name} queued in mode {self.machine.mode.value}'
        return response

    def _observation(self) -> Observation:
        now = self._now()
        def fresh(at, timeout):
            return at is not None and now - at <= timeout
        checks = {'joint_states': fresh(self.joints_at, self.cfg['joint_timeout_s']),
                  'imu': fresh(self.imu_at, self.cfg['imu_timeout_s']),
                  'controller_status': fresh(self.status_at, self.cfg['status_timeout_s'])}
        stale = tuple(name for name, ok in checks.items() if not ok)
        moving = False
        if self.cmd_vel is not None and fresh(self.cmd_vel_at, self.cfg['cmd_vel_timeout_s']):
            t = self.cmd_vel.twist
            moving = max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)) > self.cfg['cmd_vel_deadband']
        return Observation(
            now=now, estop=self.estop, supervisor_ok=self.supervisor[0],
            supervisor_reason=self.supervisor[1], supervisor_warn=self.supervisor[2], inputs_fresh=not stale, stale_inputs=stale,
            controller=self.controller, base_held=self.base_held, tilt_rad=self.tilt,
            angular_speed=self.angular_speed, fall_warning=self.fall_warning, cmd_vel_active=moving,
            command_stages=dict(self.command_stages), localization=self.localization)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        before = self.machine.mode
        actions = self.machine.step(self._observation())
        if self.machine.mode != before:
            self.get_logger().warn(f'{before.value} -> {self.machine.mode.value}: {self.machine.reason}')
            self._publish_state()
        for action in actions:
            self._execute(action)
        self._gate_cmd_vel()

    def _gate_cmd_vel(self):
        if self.cmd_vel_pub is None:
            return
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if self.machine.motion_allowed and self.cmd_vel is not None:
            out.header.frame_id = self.cmd_vel.header.frame_id
            out.twist = self.cmd_vel.twist
        if not self.dry_run:
            self.cmd_vel_pub.publish(out)

    def _log_action(self, tag, action):
        name = type(action).__name__
        self.executed.append([round(self._now(), 3), tag, name, vars(action)])
        self.get_logger().info(f'[{tag}] {name} {vars(action)}')

    def _execute(self, action):
        if self.dry_run:
            self._log_action('dry run', action)
            return
        if isinstance(action, (SelectSource, Damp)) or (
                isinstance(action, SwitchModel) and self.switch_client is None):
            # No ROS interface yet (cpp_control_node source selection, damping command, model switch).
            self._log_action('placeholder', action)
            return
        self._log_action('sent', action)
        if isinstance(action, SubmitCommand):
            self._submit(action)
        elif isinstance(action, Kick):
            self.kick_pub.publish(String(data=action.foot))
        elif isinstance(action, ResetSafety):
            self._call(self.safety_reset_client, '/safety/reset', Trigger.Request())
        elif isinstance(action, ResetController):
            if action.source == 'walk':  # The policy node has no latched fault to clear.
                self._call(self.walk_reset_client, self.cfg['walk_reset_service'], Trigger.Request())
        elif isinstance(action, ResetLocalization):
            self._call(self.localization_reset_client, self.cfg['localization_reset_service'],
                       Trigger.Request())
        elif isinstance(action, SwitchModel):
            request = SwitchPolicyModel.Request()
            request.model_name, request.restart_reference = action.model_name, action.restart_reference
            self._call(self.switch_client, '/policy/switch_model', request)

    def _submit(self, action: SubmitCommand):
        request = SubmitAbstractCommand.Request()
        now = self.get_clock().now()
        request.command.header.stamp = now.to_msg()
        request.command.valid_until = (now + Duration(seconds=0.3)).to_msg()
        request.command.request_id = action.request_id
        request.command.joint_name = action.joint_name
        request.command.operation = OPERATIONS[action.operation]
        self._call(self.command_client, '/control_core/submit_abstract_command', request)

    def _call(self, client, name, request):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} is not available; action skipped')
            return
        def done(future):
            try:
                result = future.result()
            except Exception as error:  # noqa: BLE001 - report, never crash the mode manager
                self.get_logger().error(f'{name} failed: {error!r}')
                return
            ok = getattr(result, 'success', getattr(result, 'accepted', True))
            if not ok:
                self.get_logger().warn(f'{name} refused: {getattr(result, "message", "")}')
        client.call_async(request).add_done_callback(done)

    # ------------------------------------------------------------------ outputs
    def _publish_state(self):
        state = self.machine.status()
        state.update({'dry_run': self.dry_run, 'estop': self.estop,
                      'tilt_rad': None if self.tilt is None else round(self.tilt, 3),
                      'actions': list(self.executed)})
        self.state_pub.publish(String(data=json.dumps(state, default=str)))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:  # A second SIGINT from launch during teardown.
            pass
        if rclpy.ok():
            rclpy.shutdown()
