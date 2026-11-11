"""ROS wrapper of the model-based walking controller (LIPM MPC + TSID WBC).

Inputs (simulation ground truth): /joint_states, /sim/ground_truth/odom,
/sim/ground_truth/feet, /sim/status (gantry), /safety/estop.
Commands: /walk/step_length (std_msgs/Float64, m per step; 0 = step in place),
/walk/turn (std_msgs/Float64, rad per step, + = left; 0 = straight), or with
cmd_vel_topic set a geometry_msgs/TwistStamped velocity (linear.x m/s,
angular.z rad/s) converted per step; a stale velocity stops (step 0, turn 0).
With velocity input, a command that stays at rest (no step, almost no turn)
for cmd_vel_stand_after_s finishes the current step and stands in double
support instead of stepping in place; any motion command walks again.
Kick: /walk/kick (std_msgs/String, left_foot / right_foot) kicks with that
foot at its next swing (standing: after shifting onto the other foot); the
kick state is in /walk/status ('kick', 'kicks').
Output: MotorCommand (MODE_MOTION) on /walk/desired_motor_subset, forwarded
by cpp_control_node. Any fault sends DISABLE and latches; restart the node.
"""
from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from motor_control_interfaces.msg import MotorCommand

from .gait import FAULT
from .stepping import SS
from .robot import FEET, RobotModel, load_limits
from .velocity import step_command
from .stepping import StepParams, SteppingController
from .wbc import WBCGains

MAX_STEP_M = 0.10  # largest step length validated offline (see README)
MAX_TURN_RAD = 0.15  # per step; design limit (docs/model_based_walking_design.md 4.2)


def stamp_ns(stamp) -> int:
    return stamp.sec * 10**9 + stamp.nanosec


def ordered(names, values, wanted) -> np.ndarray:
    index = {name: i for i, name in enumerate(names)}
    if any(n not in index for n in wanted) or len(values) != len(names):
        raise ValueError('Missing or malformed entries')
    return np.array([values[index[n]] for n in wanted], dtype=float)


class WalkingControllerNode(Node):
    def __init__(self):
        super().__init__('walking_controller')
        defaults = {
            'urdf_path': '', 'motor_config_file': '',
            'output_topic': '/walk/desired_motor_subset',
            'control_rate_hz': 200.0, 'input_timeout_s': 0.1,
            # sim_autostart releases the gantry only while commands stay valid > 40 ms.
            'output_lifetime_s': 0.08,
            # A single slow cycle is harmless while the last command is valid.
            'max_cycle_s': 0.03,
            'step_length_m': 0.0,
            # 'state': one cycle per physics sample, as soon as its odom, feet and
            # joint states have all arrived (lowest latency); 'timer': fixed-rate
            # timer as before.
            'trigger': 'state',
            # 'state' fallback: if no complete sample (odom, feet and joints with
            # one stamp) for this long, run once on the latest of each. Under
            # load joints and odom/feet arrived from alternating samples for
            # 50-85 ms, no cycle ran and the 80 ms commands expired (3 of 8
            # chase runs; design doc section 20). 0 = wait for a match.
            'trigger_fallback_s': 0.015,
            # Optional velocity input (e.g. /ball_chase/cmd_vel); '' = off.
            'cmd_vel_topic': '',
            'cmd_vel_timeout_s': 0.5,
            # Velocity input is capped below MAX_STEP_M: at the ROS chain's ~15 ms
            # latency 0.10 m steps fall offline after ~15 s, 0.08 m with a turn
            # falls at 20 ms; 0.06 m walks with +-0.12 rad turns at 15 and 20 ms.
            'cmd_vel_max_step_m': 0.06,
            # Turn cap for velocity input. Ball chase's heading loop (gain 1.5 rad/s
            # per rad, 0.5-1 s vision delay) limit-cycles at 0.15 rad/step: yaw swings
            # +-20 deg; at 0.08 rad/step ~6 deg (offline, same pursuit law).
            'cmd_vel_max_turn_rad': 0.08,
            # WBC contact task gain (TSID contact kp). At lift-off the stance
            # contact is pulled flat with it; 100 makes the single-support
            # force swing ~40-250 N (design doc section 17).
            'contact_kp': 100.0,
            # Turn first (velocity input): the step is full while |turn| <=
            # cmd_vel_turn_free_rad, 0 from cmd_vel_turn_stop_rad, linear in
            # between, so a large heading error is turned out in place. All
            # four chase falls came in the first approach with the full step
            # and the full turn together. 0.04 / 0.10: at the 0.08 turn cap the
            # step is a third (0.02 m); offline the worst sideways tilt fell from
            # 22 to 8.5 deg (design doc section 20). 0, 0: off.
            'cmd_vel_turn_free_rad': 0.04,
            'cmd_vel_turn_stop_rad': 0.10,
            # Stand still when the velocity command has been at rest this long
            # (step < cmd_vel_rest_step_m and |turn| < cmd_vel_rest_turn_rad per
            # step). Stepping in place drifted 3-6 mm/s forward in the ball chase,
            # which only walks forward, so it crept up to the ball. 0 = never
            # stand (step in place, the previous behaviour).
            'cmd_vel_stand_after_s': 1.0,
            'cmd_vel_rest_step_m': 0.005,
            'cmd_vel_rest_turn_rad': 0.01,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = self.params = {key: self.get_parameter(key).value for key in defaults}
        urdf = p['urdf_path'] or str(Path(get_package_share_directory('humanoid_leg_description'))
                                     / 'urdf/human_offset_corrected.urdf')
        registry = p['motor_config_file'] or str(Path(get_package_share_directory('motor_control_hybrid'))
                                                 / 'config/motors.yaml')
        self.robot = RobotModel(urdf)
        self.names = self.robot.joint_names
        limits = load_limits(registry, self.names)
        self.controller = SteppingController(self.robot, limits.torque, StepParams(
            dt=1.0 / p['control_rate_hz'], step_length_m=self._clamp(p['step_length_m']),
            gains=WBCGains(com_kp_z=100.0, com_weight=10.0, contact_kp=float(p['contact_kp']))))
        self.inputs, self.base_held, self.estop = {}, None, True
        self.started, self.fault, self.cycles = False, '', 0
        # Last 400 cycles (2 s) for /walk/status.
        self.cycle_ms, self.input_age_ms, self.joint_age_ms = (deque(maxlen=400) for _ in range(3))
        self._last_sample = None   # stamp of the last physics sample computed ('state')
        if p['trigger'] not in ('state', 'timer'):
            raise ValueError("trigger must be 'state' or 'timer'")
        # Depth 1: a cycle always uses the newest sample, never a queued older one.
        sensors = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        stream = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        safety = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(MotorCommand, p['output_topic'], stream)
        self.status_pub = self.create_publisher(String, '/walk/status', 2)
        # Stance foot at the start of each single-support phase (left_foot / right_foot), latched:
        # the swing foot clears the floor by only ~1 cm, so kinematics alone cannot tell (leg_odometry).
        self.stance_pub = self.create_publisher(
            String, '/walk/stance', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                               durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._published_stance = None
        self.create_subscription(JointState, '/joint_states', self._on_joints, sensors)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', lambda m: self._store('odom', m), sensors)
        self.create_subscription(JointState, '/sim/ground_truth/feet', self._on_feet, sensors)
        self.create_subscription(String, '/sim/status', self._sim_status, 2)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, safety)
        self.create_subscription(Float64, '/walk/step_length', self._on_step_length, 2)
        self.create_subscription(Float64, '/walk/turn', self._on_turn, 2)
        self.create_subscription(String, '/walk/kick', self._on_kick, 2)
        self.cmd_vel = None            # (step_length, turn, receipt monotonic)
        self._rest_since = None        # monotonic time the velocity command came to rest
        if p['cmd_vel_topic']:
            self.create_subscription(TwistStamped, p['cmd_vel_topic'], self._on_cmd_vel, 2)
            self.create_timer(0.1, self._check_cmd_vel)
        if p['trigger'] == 'timer':
            self.create_timer(1.0 / p['control_rate_hz'], self.tick)
        else:
            # Cycles run from _try_tick; this only catches inputs that stop arriving.
            self.create_timer(0.05, self._watchdog)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f'Model-based walking (MPC + WBC) → {p["output_topic"]}; '
                               'step length on /walk/step_length')

    @staticmethod
    def _clamp(length):
        # Backward steps were not tested; 0 .. MAX_STEP_M only.
        return float(np.clip(length, 0.0, MAX_STEP_M))

    def _on_step_length(self, msg):
        if not np.isfinite(msg.data):
            return
        self.controller.p.step_length_m = self._clamp(msg.data)

    def step_period_s(self):
        return self.controller.p.t_ds + self.controller.p.t_ss

    def _on_cmd_vel(self, msg):
        v, w = msg.twist.linear.x, msg.twist.angular.z
        if not (np.isfinite(v) and np.isfinite(w)):
            return
        step, turn = step_command(
            v, w, self.step_period_s(), min(MAX_STEP_M, float(self.params['cmd_vel_max_step_m'])),
            min(MAX_TURN_RAD, float(self.params['cmd_vel_max_turn_rad'])),
            float(self.params['cmd_vel_turn_free_rad']), float(self.params['cmd_vel_turn_stop_rad']))
        self.cmd_vel = (step, turn, time.monotonic())
        self.controller.p.step_length_m, self.controller.p.turn_rad = self.cmd_vel[:2]
        self._update_stand(step, turn)

    def _update_stand(self, step, turn):
        """Velocity input: stand after a while at rest, walk again on any motion."""
        p, c, now = self.params, self.controller, time.monotonic()
        if float(p['cmd_vel_stand_after_s']) <= 0.0:
            return
        if step < float(p['cmd_vel_rest_step_m']) and abs(turn) < float(p['cmd_vel_rest_turn_rad']):
            if self._rest_since is None:
                self._rest_since = now
            elif now - self._rest_since >= float(p['cmd_vel_stand_after_s']) and not c.stop_requested:
                c.p.step_length_m = c.p.turn_rad = 0.0
                c.request_stop()
                self.get_logger().info('Velocity command at rest: finishing the step, then standing')
        else:
            self._rest_since = None
            if c.stop_requested:
                c.resume()
                self.get_logger().info('Velocity command: walking again')

    def _check_cmd_vel(self):
        """A velocity source that stops publishing stops the robot."""
        if self.cmd_vel is not None and time.monotonic() - self.cmd_vel[2] > self.params['cmd_vel_timeout_s']:
            self.cmd_vel = None
            self.controller.p.step_length_m = self.controller.p.turn_rad = 0.0
            self.get_logger().warn('cmd_vel stale: stopping')
        if self.cmd_vel is None:
            self._update_stand(0.0, 0.0)

    def _on_kick(self, msg):
        try:
            self.controller.request_kick(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f'Kick ignored: {exc}')
            return
        self.get_logger().info(f'Kick requested: {msg.data}')

    def _on_turn(self, msg):
        if not np.isfinite(msg.data):
            return
        self.controller.p.turn_rad = float(np.clip(msg.data, -MAX_TURN_RAD, MAX_TURN_RAD))

    def _store(self, name, msg):
        self.inputs[name] = (msg, stamp_ns(msg.header.stamp), time.monotonic())

    def _on_feet(self, msg):
        self._store('feet', msg)
        self._try_tick()

    def _on_joints(self, msg):
        self._store('joints', msg)
        self._try_tick()

    def _try_tick(self):
        """'state' trigger: one cycle per physics sample, once its odom, feet and
        joints (same header stamp) have all arrived, whichever arrives last.

        Samples whose messages never all match are skipped; after
        trigger_fallback_s without a complete sample one cycle runs on the
        latest message of each (mixed samples, at most one physics step apart
        in practice), then again only on a newer one. The watchdog catches
        inputs that stop arriving.
        """
        if self.params['trigger'] != 'state':
            return
        stamps = [self.inputs[k][1] for k in ('odom', 'feet', 'joints') if k in self.inputs]
        if len(stamps) != 3:
            return
        now = time.monotonic()
        if stamps[0] == stamps[1] == stamps[2] != self._last_sample:
            self._last_sample = stamps[0]
        else:
            fallback = self.params.get('trigger_fallback_s', 0.0)
            newest = max(stamps)
            if not (fallback > 0 and now - getattr(self, '_last_tick_mono', now) > fallback
                    and newest != self._last_sample):
                return
            self._last_sample = newest
            self.mixed_ticks = getattr(self, 'mixed_ticks', 0) + 1
        self._last_tick_mono = now
        self.tick()

    def _stale(self, now_ns, now_mono):
        timeout = self.params['input_timeout_s']
        return [k for k, (_, stamp, received) in self.inputs.items()
                if (now_ns - stamp) * 1e-9 > timeout or now_mono - received > timeout]

    def _watchdog(self):
        if self.fault:
            self._send_disable()
            return
        if not self.started or self.estop:
            return
        missing = [k for k in ('joints', 'odom', 'feet') if k not in self.inputs]
        stale = missing or self._stale(self.get_clock().now().nanoseconds, time.monotonic())
        if stale:
            self._fail(f'Stale inputs: {stale}')

    def _sim_status(self, msg):
        try:
            held = json.loads(msg.data).get('base_held')
        except (ValueError, TypeError, AttributeError):
            return
        if isinstance(held, bool):
            self.base_held = held
            self.started = self.started or not held

    def _on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop and self.started:
            self._fail('Safety estop; restart the node after /sim/reset')

    def _fail(self, reason):
        if self.fault:
            return
        self.fault = reason
        self.get_logger().error(reason)
        self._send_disable()

    def _send_disable(self):
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = list(self.names)
        msg.mode = [MotorCommand.MODE_DISABLE]
        try:
            self.output.publish(msg)
        except RCLError as error:
            self.get_logger().error(f'DISABLE publication failed: {error}', throttle_duration_sec=5.0)

    def tick(self):
        if self.fault:
            self._send_disable()
            return
        if self.estop or self.base_held is None or not all(k in self.inputs for k in ('joints', 'odom', 'feet')):
            return
        begin = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        snapshot = dict(self.inputs)
        stale = self._stale(now_ns, begin)
        if stale:
            if self.started:
                self._fail(f'Stale inputs: {stale}')
            return
        try:
            joints, odom, feet = (snapshot[k][0] for k in ('joints', 'odom', 'feet'))
            pose, twist = odom.pose.pose, odom.twist.twist
            q, v = self.robot.state_from_sim(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                [twist.linear.x, twist.linear.y, twist.linear.z],
                [twist.angular.x, twist.angular.y, twist.angular.z],
                ordered(joints.name, joints.position, self.names),
                ordered(joints.name, joints.velocity, self.names))
            forces = ordered(feet.name, feet.effort, list(FEET))
            self.input_age_ms.append((now_ns - snapshot['odom'][1]) * 1e-6)
            self.joint_age_ms.append((now_ns - snapshot['joints'][1]) * 1e-6)
        except ValueError as exc:
            self._fail(str(exc))
            return
        # Latency compensation (GaitParams.predict_s): the state is this old now.
        age_s = (now_ns - snapshot['odom'][1]) * 1e-9
        command = self.controller.step(now_ns * 1e-9, q, v, forces, self.base_held, age_s=age_s)
        if command is None or self.controller.phase == FAULT:
            self._fail(f'Controller fault: {self.controller.fault}')
            return
        self._publish_stance()
        cycle = time.monotonic() - begin
        self.cycle_ms.append(cycle * 1e3)
        if cycle > self.params['max_cycle_s']:
            self._fail(f'Cycle took {cycle * 1e3:.1f} ms')
            return
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid_until.sec, msg.valid_until.nanosec = divmod(
            now_ns + int(self.params['output_lifetime_s'] * 1e9), 10**9)
        msg.joint_name = list(self.names)
        msg.mode = [MotorCommand.MODE_MOTION] * len(self.names)
        msg.position, msg.velocity = command.q_ref.tolist(), command.dq_ref.tolist()
        msg.acceleration = [0.0] * len(self.names)
        msg.torque, msg.kp, msg.kd = command.tau_ff.tolist(), command.kp.tolist(), command.kd.tolist()
        try:
            self.output.publish(msg)
        except RCLError as error:
            self._fail(f'Command publication failed: {error}')
            return
        self.cycles += 1

    def _publish_stance(self):
        c = self.controller
        stance = getattr(c, 'stance', None) if getattr(c, 'gphase', None) == SS else None
        if stance is not None and stance != self._published_stance:
            self._published_stance = stance
            try:
                self.stance_pub.publish(String(data=stance))
            except RCLError:
                pass  # Diagnostic/odometry input only; the motor command path has its own checks.

    def _publish_status(self):
        c = self.controller
        def stats(values):
            a = np.array(values) if values else np.zeros(1)
            return {'mean': float(a.mean()), 'p99': float(np.percentile(a, 99)), 'max': float(a.max())}
        status = {'state': 'fault' if self.fault else c.phase, 'fault': self.fault,
                  'base_held': self.base_held, 'estop': self.estop, 'cycles': self.cycles,
                  'step_length_m': c.p.step_length_m, 'turn_rad': c.p.turn_rad, 'standing': bool(getattr(c, 'holding', False)),
                  'cmd_vel_active': self.cmd_vel is not None,
                  'contact_kp': float(self.params['contact_kp']),
                  'mixed_ticks': getattr(self, 'mixed_ticks', 0),
                  'heading_deg': float(np.degrees(getattr(c, 'heading', 0.0))),
                  'steps': getattr(c, 'steps', 0),
                  'trigger': self.params['trigger'], 'cycle_ms': stats(self.cycle_ms),
                  # Age of the odom / joint sample at cycle start (sample stamp -> now).
                  'input_age_ms': stats(self.input_age_ms), 'joint_age_ms': stats(self.joint_age_ms),
                  'predict_ms': 1e3 * getattr(c, 'horizon_s', 0.0),
                  **{k: v for k, v in c.status.items() if k not in ('forces',)}}
        try:
            self.status_pub.publish(String(data=json.dumps(status, default=str)))
        except RCLError as error:
            self.get_logger().error(f'Status publication failed: {error}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = WalkingControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
