"""Simulated replacement for python_can_node (RobStride CAN backend).

Keeps the same relative topic/service names, message formats and command rules
so cpp_control_node, the safety supervisor and the Web monitor run unchanged:

  publishes   joint_states, motor_status, motor_telemetry, can_timing
  subscribes  motor_commands, /safety/estop
  services    motor_backend/get_feedback, motor_backend/execute_operation

Differences from hardware: no enable acknowledgement delay, no CAN latency or
frame loss, feedback is always fresh, and temperatures come from a placeholder
first-order heating model. Set-zero requests are rejected exactly like the real
backend, so cpp_control_node must run with simulation_zero_enabled=false.
"""
from __future__ import annotations

import json
import time

from motor_control_interfaces.msg import MotorCommand
from motor_control_interfaces.srv import ExecuteMotorOperation, GetMotorFeedback
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .world import SimWorld

try:
    from motor_control_hybrid.motor_configuration import MotorSettings
except ImportError:  # Raw feedback falls back to joint coordinates.
    MotorSettings = None


class SimMotorBackendNode(Node):

    def __init__(self, world: SimWorld, registry: dict) -> None:
        super().__init__('sim_motor_backend')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('ambient_temperature_c', 28.0)
        # Placeholder thermal model: dT/dt = heating * tau^2 - (T - ambient) / time_constant.
        self.declare_parameter('heating_c_per_nm2_s', 0.0005)
        self.declare_parameter('cooling_time_constant_s', 300.0)
        self.world = world
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._ambient = float(self.get_parameter('ambient_temperature_c').value)
        self._heating = float(self.get_parameter('heating_c_per_nm2_s').value)
        self._cooling = float(self.get_parameter('cooling_time_constant_s').value)

        motors = registry['motors']
        self._identity = {
            name: dict(joint_name=name, can_interface=cfg.get('can_interface', 'can1'),
                       motor_id=int(cfg['motor_id']), model=cfg.get('model'))
            for name, cfg in motors.items()}
        self._settings = {}
        if MotorSettings is not None:
            self._settings = {name: MotorSettings.from_config(cfg, registry)
                              for name, cfg in motors.items()}
        self._temperature = {name: self._ambient for name in motors}
        self._last_sample = None
        self._last_sample_mono = time.monotonic()
        self._last_tick = time.monotonic()
        self._rejected_warn = 0.0
        self._timing = dict(rx_frames=0, commands_received=0, commands_rejected=0)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_status_pub = self.create_publisher(String, 'motor_status', 10)
        self.telemetry_pub = self.create_publisher(String, 'motor_telemetry', 10)
        self.timing_pub = self.create_publisher(String, 'can_timing', 1)
        self.create_subscription(MotorCommand, 'motor_commands', self._command_callback, 10)
        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop', self._estop_callback, estop_qos)
        self.create_service(GetMotorFeedback, 'motor_backend/get_feedback', self._get_feedback)
        self.create_service(ExecuteMotorOperation, 'motor_backend/execute_operation',
                            self._execute_operation)
        self.create_timer(1.0 / rate if rate > 0.0 else 0.02, self._publish_states)
        self.create_timer(1.0, self._publish_timing)
        self.get_logger().warn(
            f'SIMULATED motor backend for {len(motors)} joints; no CAN hardware is used')

    # --------------------------------------------------------------- inputs
    def _estop_callback(self, msg: Bool) -> None:
        with self.world.lock:
            was_active = self.world.motors.estop_active
            self.world.motors.set_estop(msg.data)
        if msg.data and not was_active:
            self.get_logger().fatal('Safety estop asserted; rejecting commands and disabling motors')
        elif was_active and not msg.data:
            self.get_logger().warn('Safety estop reset; motors remain disabled until explicitly enabled')

    def _command_callback(self, msg: MotorCommand) -> None:
        now_mono = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid_until = msg.valid_until.sec + msg.valid_until.nanosec * 1e-9
        with self.world.lock:
            rejected = self.world.motors.submit(
                list(msg.joint_name), list(msg.mode), list(msg.position), list(msg.velocity),
                list(msg.torque), list(msg.kp), list(msg.kd),
                stamp_s=stamp, valid_until_s=valid_until, now_ros_s=now_ros, now_mono_s=now_mono)
        self._timing['commands_received'] += 1
        if rejected:
            self._timing['commands_rejected'] += 1
            if now_mono - self._rejected_warn >= 1.0:
                self._rejected_warn = now_mono
                self.get_logger().warn(
                    f'Rejecting stale, invalid or non-finite command for {", ".join(rejected)}')

    # -------------------------------------------------------------- outputs
    def _publish_states(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        sample = self.world.joint_snapshot()
        with self.world.lock:
            events = self.world.motors.pop_events()
        for event in events:
            self.get_logger().info(event)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = sample.names
        msg.position = [float(x) for x in sample.position]
        msg.velocity = [float(x) for x in sample.velocity]
        msg.effort = [float(x) for x in sample.effort]
        self.joint_state_pub.publish(msg)
        self._last_sample = (msg.header.stamp, sample)
        self._last_sample_mono = now
        self._timing['rx_frames'] += len(sample.names)

        status, telemetry = {}, []
        source_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for index, (name, torque) in enumerate(zip(sample.names, sample.effort)):
            temp = self._temperature[name]
            temp += dt * (self._heating * torque * torque - (temp - self._ambient) / self._cooling)
            self._temperature[name] = temp
            status[str(index)] = {'temperature': float(temp), 'torque': float(torque)}
            telemetry.append(dict(self._identity[name], source_stamp=source_stamp,
                                  temperature_c=float(temp)))
        self.motor_status_pub.publish(String(data=json.dumps(status)))
        self.telemetry_pub.publish(String(data=json.dumps(
            dict(schema_version=1, motors=telemetry), allow_nan=False)))

    def _publish_timing(self) -> None:
        payload = dict(self._timing, simulated=True, transport_fault=False, pending_commands=0,
                       physics_overruns=self.world.overruns)
        payload['interfaces'] = {
            iface: {'tx_submitted': 0, 'tx_errors': 0}
            for iface in sorted({ident['can_interface'] for ident in self._identity.values()})}
        self.timing_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------- services
    def _get_feedback(self, request, response):
        """Cached feedback in raw motor coordinates, like the CAN backend."""
        feedback = response.feedback
        feedback.joint_name = request.joint_name
        for name in ('age_s', 'position_rad', 'velocity_radps', 'torque_nm', 'temperature_c'):
            setattr(feedback, name, float('nan'))
        identity = self._identity.get(request.joint_name)
        if identity is None:
            response.message = 'Unknown joint'
            return response
        feedback.can_interface = identity['can_interface']
        feedback.motor_id = identity['motor_id']
        feedback.model = str(identity['model'])
        if self._last_sample is None:
            response.message = 'No simulated feedback published yet'
            return response
        stamp, sample = self._last_sample
        index = sample.names.index(request.joint_name)
        position = float(sample.position[index])
        velocity = float(sample.velocity[index])
        torque = float(sample.effort[index])
        settings = self._settings.get(request.joint_name)
        if settings is not None:
            # Joint -> motor coordinates (direction, offset, ankle linkage position).
            raw = settings.command(dict(position=position, velocity=velocity, acceleration=0.0,
                                        torque=torque, kp=0.0, kd=0.0))
            position, velocity, torque = raw['position'], raw['velocity'], raw['torque']
        feedback.available = True
        feedback.age_s = time.monotonic() - self._last_sample_mono
        feedback.fresh = feedback.age_s <= 0.1
        feedback.stamp = stamp
        feedback.position_rad = position
        feedback.velocity_radps = velocity
        feedback.torque_nm = torque
        feedback.temperature_c = self._temperature[request.joint_name]
        with self.world.lock:
            feedback.status_mode = 2 if self.world.motors.is_enabled(request.joint_name) else 0
        response.success = feedback.fresh
        response.message = 'SIMULATED feedback; no CAN hardware'
        return response

    def _execute_operation(self, request, response):
        """Reject zeroing for the same reasons the hardware backend does."""
        response.request_id = request.request_id
        response.accepted = False
        response.state = ExecuteMotorOperation.Response.REJECTED
        if not request.request_id or len(request.request_id) > 128:
            response.message = 'request_id must contain 1–128 characters'
        elif request.joint_name not in self._identity:
            response.message = 'Unknown joint'
        elif request.operation != ExecuteMotorOperation.Request.SET_MECHANICAL_ZERO:
            response.message = 'Unsupported operation; use typed ROS constants, not CAN frame numbers'
        elif self.world.motors.estop_active:
            response.message = 'Safety estop is active'
        else:
            response.message = ('Not ready: zero transaction is not implemented by the hardware '
                                'backend, so the simulator rejects it too')
        return response
