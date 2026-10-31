from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import yaml

from motor_control_interfaces.msg import MotorCommand
from .bridge import ContactSelector, fresh, ordered, stamp_ns
from .controller import WBCController


class WBCControlNode(Node):
    def __init__(self):
        super().__init__('wbc_control_node')
        defaults = {
            'urdf_path': '', 'motor_config_file': '',
            'policy_topic': '/policy/desired_motor_subset',
            'output_topic': '/wbc/desired_motor_subset',
            'joint_states_topic': '/joint_states',
            'odom_topic': '/sim/ground_truth/odom',
            'feet_topic': '/sim/ground_truth/feet',
            'control_rate_hz': 100.0, 'input_timeout_s': .1,
            'max_input_skew_s': .04, 'max_compute_s': .02,
            'output_lifetime_s': .08, 'posture_weight': 1.0,
            'contact_mode': 'policy', 'com_reference_mode': 'policy',
            'tracking_mode': 'policy_impedance', 'max_policy_torque_correction_nm': .02,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.params = {key: self.get_parameter(key).value for key in defaults}
        p = self.params
        for key in ('control_rate_hz', 'input_timeout_s', 'max_input_skew_s',
                    'max_compute_s', 'output_lifetime_s', 'posture_weight', 'max_policy_torque_correction_nm'):
            if not np.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if p['policy_topic'] == p['output_topic']:
            raise ValueError('Input and output topics must differ')
        if p['contact_mode'] not in ('policy', 'double_support'):
            raise ValueError('contact_mode must be policy or double_support')
        if p['com_reference_mode'] not in ('policy', 'hold'):
            raise ValueError('com_reference_mode must be policy or hold')
        urdf = p['urdf_path'] or str(Path(get_package_share_directory('humanoid_leg_description')) /
                                      'urdf/human_offset_corrected.urdf')
        registry = p['motor_config_file'] or str(Path(get_package_share_directory('motor_control_hybrid')) /
                                               'config/motors.yaml')
        self.controller = WBCController(Path(urdf))
        if self.controller.dynamics is None:
            raise ValueError('Aligned simulation model and dynamics metadata are required')
        self.names = self.controller.joint_names
        config = yaml.safe_load(Path(registry).read_text())['motor_control_node']['ros__parameters']
        motors = config['motors']
        peak = {'rs-02': 17., 'rs-03': 60., 'rs-04': 120.}
        self.limits = np.array([
            motors[n]['max_torque_nm'] if motors[n].get('max_torque_nm') is not None
            else peak[motors[n]['model']] for n in self.names
        ], dtype=float)
        self.lower = np.array([motors[n]['min_position'] for n in self.names])
        self.upper = np.array([motors[n]['max_position'] for n in self.names])
        self.controller.set_torque_limits(self.limits)
        self.controller.form.updateTaskWeight('posture', float(p['posture_weight']))
        if p['tracking_mode'] not in ('policy_impedance', 'policy_torque', 'posture'):
            raise ValueError('tracking_mode must be policy_impedance, policy_torque or posture')
        if p['tracking_mode'] in ('policy_impedance', 'policy_torque'):
            self.controller.enable_policy_torque_tracking(float(p['max_policy_torque_correction_nm']))
        self.last_nominal_torque = []
        self.base_held = True
        self.target_data = self.controller.model.createData()
        self.selector = ContactSelector()
        self.inputs = {}
        self.estop = True
        self.fault = ''
        self.started = False
        self.state = 'waiting_for_inputs'
        self.last_policy_stamp = 0
        self.solved_count = 0
        self.last_compute_ms = 0.
        self.last_torque = []
        stream = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        safety = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(MotorCommand, p['output_topic'], stream)
        self.status = self.create_publisher(String, '/wbc/status', 10)
        self.create_subscription(MotorCommand, p['policy_topic'], self._policy, stream)
        self.create_subscription(JointState, p['joint_states_topic'],
                                 lambda msg: self._store('joints', msg), stream)
        self.create_subscription(Odometry, p['odom_topic'],
                                 lambda msg: self._store('odom', msg), stream)
        self.create_subscription(JointState, p['feet_topic'],
                                 lambda msg: self._store('feet', msg), stream)
        self.create_subscription(String, '/sim/status', self._sim_status, 1)
        self.create_subscription(Bool, '/safety/estop', self._estop, safety)
        self.create_timer(1. / p['control_rate_hz'], self.tick)
        self.create_timer(.5, self._status)
        self.get_logger().info(f'Policy → WBC → {p["output_topic"]}; awaiting state and released safety')

    def _store(self, name, msg):
        self.inputs[name] = (msg, stamp_ns(msg.header.stamp), time.monotonic())

    def _policy(self, msg):
        source = stamp_ns(msg.header.stamp)
        if MotorCommand.MODE_DISABLE in msg.mode:
            self._fail('Upstream disable')
            return
        if source <= self.last_policy_stamp:
            return
        try:
            if set(msg.joint_name) != set(self.names):
                raise ValueError('Policy must command exactly all leg joints')
            ordered(msg.joint_name, msg.position, self.names)
            if len(msg.mode) not in (1, len(self.names)) or any(m != MotorCommand.MODE_MOTION for m in msg.mode):
                raise ValueError('Policy input must use MODE_MOTION')
            if stamp_ns(msg.valid_until) <= source:
                raise ValueError('Policy input needs an explicit deadline')
        except ValueError as exc:
            self._fail(str(exc))
            return
        self.last_policy_stamp = source
        self._store('policy', msg)

    def _sim_status(self, msg):
        try:
            value = json.loads(msg.data).get('base_held')
        except (ValueError, TypeError):
            return
        if isinstance(value, bool):
            self.base_held = value
            if not value:
                self.started = True

    def _estop(self, msg):
        self.estop = msg.data
        if self.estop and self.started:
            self._fail('Safety estop; restart node after reset')

    def _fail(self, reason):
        if self.fault:
            return
        self.fault = reason
        self.state = 'fault'
        disable = MotorCommand()
        disable.header.stamp = self.get_clock().now().to_msg()
        disable.joint_name = list(self.names)
        disable.mode = [MotorCommand.MODE_DISABLE]
        self.output.publish(disable)
        self.get_logger().error(reason)

    def _status(self):
        self.status.publish(String(data=json.dumps({
            'state': self.state, 'fault': self.fault, 'solved_count': self.solved_count,
            'compute_ms': self.last_compute_ms, 'solve_ms': self.controller.last_solve_ms,
            'contacts': list(self.controller.contacts),
            'joint_names': self.names, 'torques': self.last_torque,
            'nominal_torques': self.last_nominal_torque, 'tracking_mode': self.params['tracking_mode'],
            'base_held': self.base_held,
        })))

    def tick(self):
        if self.fault or self.estop:
            return
        if not all(name in self.inputs for name in ('policy', 'joints', 'odom', 'feet')):
            return
        now_ns = self.get_clock().now().nanoseconds
        begin = time.monotonic()
        p = self.params
        snapshot = dict(self.inputs)
        if not all(fresh(stamp, received, now_ns, begin, p['input_timeout_s'])
                   for _, stamp, received in snapshot.values()):
            if self.started:
                self._fail('Input source/receipt timeout')
            return
        stamps = [entry[1] for entry in snapshot.values()]
        if (max(stamps) - min(stamps)) * 1e-9 > p['max_input_skew_s']:
            self.state = 'waiting_for_aligned_inputs'
            return
        policy = snapshot['policy'][0]
        deadline = min(stamp_ns(policy.valid_until),
                       *(stamp + int(p['input_timeout_s'] * 1e9) for stamp in stamps),
                       now_ns + int(p['output_lifetime_s'] * 1e9))
        if deadline <= now_ns:
            if self.started:
                self._fail('Policy deadline expired')
            return
        try:
            joints, odom, feet = (snapshot[name][0] for name in ('joints', 'odom', 'feet'))
            if odom.header.frame_id != 'world' or odom.child_frame_id != 'floating_base' or feet.header.frame_id != 'world':
                raise ValueError('Expected simulation world/floating_base frames')
            positions = ordered(joints.name, joints.position, self.names)
            velocities = ordered(joints.name, joints.velocity, self.names)
            q_ref = ordered(policy.joint_name, policy.position, self.names)
            if np.any(q_ref < self.lower - 1e-6) or np.any(q_ref > self.upper + 1e-6):
                raise ValueError('Policy target exceeds joint limits')
            pose, twist = odom.pose.pose, odom.twist.twist
            q, dq = self.controller.state_from_sim(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                [twist.linear.x, twist.linear.y, twist.linear.z],
                [twist.angular.x, twist.angular.y, twist.angular.z],
                positions, velocities,
            )
            foot_names = ['left_foot', 'right_foot']
            heights = ordered(feet.name, feet.position, foot_names)
            forces = ordered(feet.name, feet.effort, foot_names)
            target = q.copy()
            target[7:] = q_ref
            pin.framesForwardKinematics(self.controller.model, self.target_data, target)
            target_heights = []
            for name in foot_names:
                frame = self.target_data.oMf[self.controller.model.getFrameId(name)]
                points = np.array(self.controller.dynamics['contact_points'][name])
                target_heights.append(np.min((frame.rotation @ points + frame.translation[:, None])[2]))
            active = (set(foot_names) if p['contact_mode'] == 'double_support' else
                      self.selector.update(heights, forces, target_heights, begin))
            self.controller.update_contacts(active, q)
            if p['com_reference_mode'] == 'policy':
                com = pin.centerOfMass(self.controller.model, self.target_data, target).copy()
                self.controller.set_com_reference(com)
            nominal = None
            if p.get('tracking_mode', 'posture') in ('policy_impedance', 'policy_torque'):
                kp = ordered(policy.joint_name, policy.kp, self.names)
                kd = ordered(policy.joint_name, policy.kd, self.names)
                velocity_ref = ordered(policy.joint_name, policy.velocity, self.names)
                feedforward = ordered(policy.joint_name, policy.torque, self.names)
                if np.any(kp < 0) or np.any(kd < 0):
                    raise ValueError('Policy gains must be nonnegative')
                pd_torque = kp * (q_ref - positions) + kd * (velocity_ref - velocities)
                nominal = pd_torque + feedforward
                nominal = np.clip(nominal, -self.limits, self.limits)
            tau = self.controller.compute(q, dq, q_ref, now_ns * 1e-9, torque_ref=nominal)
            if np.any(np.abs(tau) > self.limits + 1e-5):
                raise RuntimeError(f'WBC torque exceeds configured bounds: excess={np.max(np.abs(tau) - self.limits):.9g}, tau={tau.tolist()}')
            if p.get('tracking_mode') == 'policy_impedance':
                correction = tau - nominal + feedforward
                if np.any(np.abs(correction) > self.limits + 1e-5):
                    raise RuntimeError('WBC impedance correction exceeds backend feedforward bounds')
        except (ValueError, RuntimeError) as exc:
            self._fail(str(exc))
            return
        self.last_compute_ms = (time.monotonic() - begin) * 1000
        if self.last_compute_ms > p['max_compute_s'] * 1000 or deadline <= self.get_clock().now().nanoseconds:
            self._fail(f'WBC result missed its deadline: total_ms={self.last_compute_ms:.3f}, solve_ms={self.controller.last_solve_ms:.3f}')
            return
        command = MotorCommand()
        command.header = deepcopy(policy.header)
        command.valid_until.sec, command.valid_until.nanosec = divmod(deadline, 10**9)
        command.joint_name = list(self.names)
        command.mode = [MotorCommand.MODE_MOTION] * len(self.names)
        command.position = positions.tolist()
        command.velocity = [0.] * len(self.names)
        command.acceleration = [0.] * len(self.names)
        command.torque = tau.tolist()
        command.kp = [0.] * len(self.names)
        command.kd = [0.] * len(self.names)
        if p.get('tracking_mode') == 'policy_impedance':
            command.position = q_ref.tolist()
            command.velocity = velocity_ref.tolist()
            command.kp = kp.tolist()
            command.kd = kd.tolist()
            command.torque = correction.tolist()
        if getattr(self, 'base_held', False):
            command.position = positions.tolist()
            command.velocity = [0.] * len(self.names)
            command.kp = command.kd = command.torque = [0.] * len(self.names)
        self.output.publish(command)
        self.started = self.started or not getattr(self, 'base_held', False)
        self.state = 'arming_on_gantry' if getattr(self, 'base_held', False) else 'running'
        self.solved_count += 1
        self.last_torque = tau.tolist()
        self.last_nominal_torque = [] if nominal is None else nominal.tolist()


def main(args=None):
    rclpy.init(args=args)
    node = WBCControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
