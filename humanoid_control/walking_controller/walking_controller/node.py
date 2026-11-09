"""ROS wrapper of the model-based walking controller (LIPM MPC + TSID WBC).

Inputs (simulation ground truth): /joint_states, /sim/ground_truth/odom,
/sim/ground_truth/feet, /sim/status (gantry), /safety/estop.
Command: /walk/step_length (std_msgs/Float64, m per step; 0 = step in place).
Output: MotorCommand (MODE_MOTION) on /walk/desired_motor_subset, forwarded
by cpp_control_node. Any fault sends DISABLE and latches; restart the node.
"""
from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from motor_control_interfaces.msg import MotorCommand

from .gait import FAULT
from .robot import FEET, RobotModel, load_limits
from .stepping import StepParams, SteppingController
from .wbc import WBCGains

MAX_STEP_M = 0.10  # largest step length validated offline (see README)


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
            gains=WBCGains(com_kp_z=100.0, com_weight=10.0)))
        self.inputs, self.base_held, self.estop = {}, None, True
        self.started, self.fault, self.cycles, self.cycle_ms = False, '', 0, []
        stream = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        safety = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(MotorCommand, p['output_topic'], stream)
        self.status_pub = self.create_publisher(String, '/walk/status', 2)
        self.create_subscription(JointState, '/joint_states', lambda m: self._store('joints', m), stream)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', lambda m: self._store('odom', m), stream)
        self.create_subscription(JointState, '/sim/ground_truth/feet', lambda m: self._store('feet', m), stream)
        self.create_subscription(String, '/sim/status', self._sim_status, 2)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, safety)
        self.create_subscription(Float64, '/walk/step_length', self._on_step_length, 2)
        self.create_timer(1.0 / p['control_rate_hz'], self.tick)
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

    def _store(self, name, msg):
        self.inputs[name] = (msg, stamp_ns(msg.header.stamp), time.monotonic())

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
        timeout = self.params['input_timeout_s']
        snapshot = dict(self.inputs)
        stale = [k for k, (_, stamp, received) in snapshot.items()
                 if (now_ns - stamp) * 1e-9 > timeout or begin - received > timeout]
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
        except ValueError as exc:
            self._fail(str(exc))
            return
        command = self.controller.step(now_ns * 1e-9, q, v, forces, self.base_held)
        if command is None or self.controller.phase == FAULT:
            self._fail(f'Controller fault: {self.controller.fault}')
            return
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

    def _publish_status(self):
        c = self.controller
        cycles = np.array(self.cycle_ms[-400:]) if self.cycle_ms else np.zeros(1)
        status = {'state': 'fault' if self.fault else c.phase, 'fault': self.fault,
                  'base_held': self.base_held, 'estop': self.estop, 'cycles': self.cycles,
                  'step_length_m': c.p.step_length_m, 'steps': getattr(c, 'steps', 0),
                  'cycle_ms': {'mean': float(cycles.mean()), 'p99': float(np.percentile(cycles, 99)),
                               'max': float(cycles.max())},
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
