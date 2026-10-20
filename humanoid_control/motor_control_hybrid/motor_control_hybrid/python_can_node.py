#!/usr/bin/env python3
"""ROS 2 RobStride adapter with event-driven TX and independent per-bus RX.

TX submits zero-wait SocketCAN frames; it never waits for motor feedback.
Model timing remains 50 Hz. Fresh commands wake TX immediately, while hold-last
retransmission uses a monotonic deadline without adding a sleep after work.
"""

import json
import math
import os
import queue
import struct
import threading
import time
from typing import Dict, Tuple

import numpy as np
import rclpy
import yaml
from motor_control_interfaces.msg import MotorCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus
from motor_control_hybrid.motor_configuration import validate_registry
from motor_control_hybrid.can_transport import NonblockingRobstrideBus, LatestCommandMailbox
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class PythonCanNode(Node):
    def __init__(self):
        super().__init__('python_can_node')

        self.declare_parameter('motor_config_file', '')
        self.declare_parameter('can_rt_enabled', False)
        self.declare_parameter('can_tx_rt_priority', 20)
        self.declare_parameter('can_rx_rt_priority', 21)
        self.can_rt_enabled = self.get_parameter('can_rt_enabled').value
        self.can_tx_rt_priority = self.get_parameter('can_tx_rt_priority').value
        self.can_rx_rt_priority = self.get_parameter('can_rx_rt_priority').value
        for priority in (self.can_tx_rt_priority, self.can_rx_rt_priority):
            if not 1 <= priority <= 99:
                raise ValueError('CAN RT priorities must be between 1 and 99')
        self.declare_parameter('publish_rate_hz', 50.0)
        # Legacy launch compatibility only: RX now continuously consumes frames.
        self.declare_parameter('feedback_poll_hz', 50.0)
        self.declare_parameter('feedback_poll_when_idle', True)
        self.declare_parameter('feedback_timeout_s', 0.1)
        # A transport-local watchdog: upstream process death must not leave MIT
        # targets being retransmitted forever. Zero/negative disables are forbidden.
        self.declare_parameter('command_timeout_s', 0.5)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        if not math.isfinite(self.command_timeout_s) or self.command_timeout_s <= 0:
            raise ValueError('command_timeout_s must be finite and positive')
        self.feedback_timeout_s = float(self.get_parameter('feedback_timeout_s').value)
        if not 0 < self.feedback_timeout_s <= 1.0:
            raise ValueError('feedback_timeout_s must be in (0, 1]')

        cfg_path = self.get_parameter('motor_config_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        if not math.isfinite(publish_rate) or publish_rate <= 0:
            raise ValueError('publish_rate_hz must be finite and positive')

        self.default_iface = 'can0'
        self.default_master = 255
        self.default_kp = 10.0
        self.default_kd = 0.2
        self.max_vel_rad_s = 4.5
        self.freq_hz = 0.8

        self.buses: Dict[str, RobstrideBus] = {}
        self.bus_locks: Dict[str, threading.Lock] = {}
        self.bus_by_joint: Dict[str, RobstrideBus] = {}
        self.motor_name_by_joint: Dict[str, str] = {}
        self.motor_index_map: Dict[str, int] = {}
        self.current_mode_by_joint: Dict[str, int | None] = {}
        self.estop_active = threading.Event()
        self._estop_disable_pending = threading.Event()

        self._load_config_and_connect(cfg_path)

        self.telemetry_stamps = {}
        self.state_buffer: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.state_lock = threading.Lock()

        self.command_queue = LatestCommandMailbox()
        self.active_commands = {}
        self.active_command_lock = threading.Lock()
        # These watchdog structures belong exclusively to the CAN worker thread.
        self.command_deadlines = {}
        self.inhibited_since = {joint: float('-inf') for joint in self.motor_name_by_joint}
        self.pending_disable = set()
        self.enable_pending = {}
        self.enable_reports = {}
        self.enable_report_lock = threading.Lock()
        self._awaiting_enable_ack = threading.Event()
        self.tx_period_s = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self._tx_wakeup = threading.Event()
        self.transport_fault = threading.Event()
        self.running = True
        self.timing = {'tx_cycles': 0, 'tx_max_work_ms': 0.0, 'tx_errors': 0,
                       'rx_frames': 0, 'rx_ignored': 0}
        self._joint_by_motor = {
            (bus.channel, self.motor_name_by_joint[joint]): joint
            for joint, bus in self.bus_by_joint.items()}
        self.can_rx_threads = [threading.Thread(
            target=self._can_rx_loop, args=(bus,), daemon=True,
            name=f'CAN_RX_{bus.channel}') for bus in self.buses.values()]

        self.can_tx_thread = threading.Thread(
            target=self._can_tx_loop,
            daemon=True,
            name='CAN_TX_Thread',
        )

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_status_pub = self.create_publisher(String, 'motor_status', 10)
        self.telemetry_pub = self.create_publisher(String, 'motor_telemetry', 10)
        self.timing_pub = self.create_publisher(String, 'can_timing', 1)
        self.timing_timer = self.create_timer(1.0, self._publish_timing)
        self.cmd_sub = self.create_subscription(
            MotorCommand,
            'motor_commands',
            self._cmd_callback,
            10,
        )
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            '/safety/estop',
            self._safety_estop_callback,
            estop_qos,
        )

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self._publish_states)
        for thread in self.can_rx_threads:
            thread.start()
        self.can_tx_thread.start()

        self.get_logger().info(
            f'Python CAN node started with {len(self.motor_name_by_joint)} motors '
            f'on {len(self.buses)} CAN interface(s)'
        )

    def _load_config_and_connect(self, cfg_path: str):
        if not cfg_path or not os.path.exists(cfg_path):
            self.get_logger().warn(f'No motor config file found: {cfg_path}')
            return

        self.get_logger().info(f'Loading motor config from: {cfg_path}')
        with open(cfg_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        if 'motor_control_node' in data:
            params = data['motor_control_node'].get('ros__parameters', {})
        else:
            params = data

        self.default_iface = params.get('default_can_interface', 'can0')
        self.default_master = int(params.get('default_master_id', 255))
        self.default_kp = float(params.get('KP', 10.0))
        self.default_kd = float(params.get('KD', 0.2))
        self.max_vel_rad_s = float(params.get('MAX_VEL_RAD_S', 4.5))
        self.freq_hz = float(params.get('FREQ_HZ', 0.8))

        self.motor_settings = validate_registry(params)
        motors_cfg = params.get('motors', {}) or {}
        self.motor_identity = {
            name: dict(joint_name=name,
                       can_interface=cfg.get('can_interface', self.default_iface),
                       motor_id=cfg['motor_id'], model=cfg.get('model'))
            for name, cfg in motors_cfg.items()
        }
        motors_by_iface: Dict[str, Dict[str, Motor]] = {}

        for motor_index, (joint_name, cfg) in enumerate(motors_cfg.items()):
            iface = cfg.get('can_interface', self.default_iface)
            motor_id = int(cfg['motor_id'])
            model = str(cfg.get('model', 'rs-03'))
            motor_name = f'motor_{motor_id}'

            motors_by_iface.setdefault(iface, {})[motor_name] = Motor(
                id=motor_id,
                model=model,
            )
            self.motor_name_by_joint[joint_name] = motor_name
            self.motor_index_map[joint_name] = motor_index
            self.current_mode_by_joint[joint_name] = None

            self.get_logger().info(
                f'Configured motor[{motor_index}]: {joint_name} on {iface}, '
                f'motor_id={motor_id}, model={model}'
            )

        for iface, motors in motors_by_iface.items():
            try:
                bus = NonblockingRobstrideBus(iface, motors, {})
                bus.host_id = next(
                    cfg.get('master_id', self.default_master) for cfg in motors_cfg.values()
                    if cfg.get('can_interface', self.default_iface) == iface)
                bus.connect(handshake=True)
                self.buses[iface] = bus
                self.bus_locks[iface] = threading.Lock()
                for joint_name, motor_name in self.motor_name_by_joint.items():
                    if motors_cfg[joint_name].get('can_interface', self.default_iface) == iface:
                        self.bus_by_joint[joint_name] = bus
                self.get_logger().info(
                    f'Connected RobstrideBus on {iface} with {len(motors)} motor(s)'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to connect {iface}: {e}')

    def _clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def _set_mode_raw(self, bus: RobstrideBus, motor_name: str, mode: int):
        device_id = bus.motors[motor_name].id
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack('<bBH', int(mode), 0, 0)
        data = struct.pack('<HH', param_id, 0x00) + value_buffer
        bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, device_id, data)

    def _write_parameter(
        self,
        bus: RobstrideBus,
        joint_name: str,
        motor_name: str,
        parameter_type: tuple,
        value,
    ):
        device_id = bus.motors[motor_name].id
        param_id, param_dtype, param_name = parameter_type

        match param_dtype:
            case np.uint8:
                value_buffer = struct.pack('<BBH', int(value), 0, 0)
            case np.int8:
                value_buffer = struct.pack('<bBH', int(value), 0, 0)
            case np.uint16:
                value_buffer = struct.pack('<HH', int(value), 0)
            case np.int16:
                value_buffer = struct.pack('<hH', int(value), 0)
            case np.uint32:
                value_buffer = struct.pack('<L', int(value))
            case np.int32:
                value_buffer = struct.pack('<l', int(value))
            case np.float32:
                value_buffer = struct.pack('<f', float(value))
            case _:
                raise ValueError(f'Unsupported parameter type of {param_name}: {param_dtype}')

        data = struct.pack('<HH', param_id, 0x00) + value_buffer
        bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, device_id, data)

        return True  # Submitted, not acknowledged; RX owns all feedback.

    def _update_state(self, joint_name: str, pos: float, vel: float, tq: float, temp: float,
                      received_at=None):
        pos, vel, tq = self.motor_settings[joint_name].feedback(pos, vel, tq)
        now = time.monotonic()
        received_at = now if received_at is None else received_at
        source_stamp = self.get_clock().now().nanoseconds * 1e-9 - max(0.0, now - received_at)
        with self.state_lock:
            self.telemetry_stamps[joint_name] = source_stamp
            self.state_buffer[joint_name] = (pos, vel, tq, temp, received_at)

    def _configure_can_scheduling(self, role, priority):
        tid = threading.get_native_id()
        if not self.can_rt_enabled:
            self.get_logger().info(f'CAN {role} TID={tid}: existing scheduling')
            return
        try:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(priority))
            self.get_logger().info(f'CAN {role} TID={tid}: SCHED_RR priority={priority}')
        except (OSError, AttributeError) as exc:
            self.get_logger().warning(
                f'CAN {role} TID={tid}: cannot set SCHED_RR priority={priority}: '
                f'{exc}; continuing with existing scheduling')

    def _can_rx_loop(self, bus):
        """The only receiver for this interface; a missing motor cannot block TX.

        recv() has a bounded idle wait on this separate thread and returns as
        soon as a frame arrives. Kernel frame age is preserved in host monotonic
        seconds so queued feedback is not relabelled as a fresh measurement.
        """
        self._configure_can_scheduling(f'RX {bus.channel}', self.can_rx_rt_priority)
        while self.running and rclpy.ok():
            try:
                frame = bus.channel_handler.recv(timeout=0.01)
                if frame is None:
                    continue
                sample = bus.decode_feedback(frame)
                if sample is None:
                    self.timing['rx_ignored'] += 1
                    continue
                self.timing['rx_frames'] += 1
                if sample.fault_bits:
                    raise RuntimeError(f'{sample.motor_name}: status fault bits 0x{sample.fault_bits:02x}')
                age = max(0.0, time.time() - sample.timestamp) if sample.timestamp > 0 else 0.0
                if age > self.feedback_timeout_s:
                    continue
                joint = self._joint_by_motor[(bus.channel, sample.motor_name)]
                if self._awaiting_enable_ack.is_set():
                    with self.enable_report_lock:
                        self.enable_reports[joint] = (time.monotonic() - age, sample.mode)
                    self._tx_wakeup.set()
                self._update_state(joint, sample.position, sample.velocity,
                                   sample.torque, sample.temperature, time.monotonic() - age)
            except Exception as exc:
                # A local transport/motor fault cannot be cleared by a healthy
                # supervisor publishing false; restart after diagnosis to reset.
                if not self.transport_fault.is_set():
                    self.transport_fault.set()
                    self._estop_disable_pending.set()
                    self._tx_wakeup.set()
                self.get_logger().error(f'CAN RX fault on {bus.channel}: {exc}', throttle_duration_sec=1.0)
                time.sleep(0.01)  # RX error backoff only; TX is independent.

    def _can_tx_loop(self):
        self._configure_can_scheduling('TX', self.can_tx_rt_priority)
        self.get_logger().info('CAN TX thread started; feedback never blocks TX')
        while self.running and rclpy.ok():
            begin = time.monotonic()
            self._tx_wakeup.clear()
            self._expire_commands()
            self._drain_command_queue()
            self._process_enable_reports()
            if self.estop_active.is_set() or self.transport_fault.is_set():
                self._enforce_estop()
            else:
                with self.active_command_lock:
                    active = list(self.active_commands.values())
                for cmd in active:
                    self._send_active_command(cmd)
            end = time.monotonic()
            self.timing['tx_cycles'] += 1
            self.timing['tx_max_work_ms'] = max(self.timing['tx_max_work_ms'], (end - begin) * 1000)
            # Wait only the remainder of the period, or wake immediately on a
            # new command, stop request, or an earlier per-joint deadline.
            wake_at = min([begin + self.tx_period_s] + list(self.command_deadlines.values()))
            self._tx_wakeup.wait(max(0.0, wake_at - end))

    def _publish_timing(self):
        payload = dict(self.timing)
        payload.update(pending_commands=self.command_queue.qsize(),
                       transport_fault=self.transport_fault.is_set(),
                       repeat_hz=1.0 / self.tx_period_s)
        payload['interfaces'] = {name: {'tx_submitted': bus.tx_frames, 'tx_errors': bus.tx_errors}
                                 for name, bus in self.buses.items()}
        self.timing_pub.publish(String(data=json.dumps(payload)))

    def _process_enable_reports(self):
        """Complete enables asynchronously on fresh healthy Motor-mode status.

        Type-2 status mode 2 means Motor/run, distinct from run_mode parameter 2
        (velocity control). No motion is sent while enable confirmation is pending.
        """
        with self.enable_report_lock:
            reports = self.enable_reports.copy()
            self.enable_reports.clear()
        for joint, (received_at, mode) in reports.items():
            submitted_at = self.enable_pending.get(joint)
            if (submitted_at is not None and received_at >= submitted_at and mode == 2
                    and time.monotonic() < self.command_deadlines.get(joint, 0)):
                self.enable_pending.pop(joint, None)
                self.inhibited_since.pop(joint, None)
        if not self.enable_pending:
            self._awaiting_enable_ack.clear()

    def _enforce_estop(self):
        if not self._estop_disable_pending.is_set():
            return
        self.command_deadlines.clear()
        self.enable_pending.clear()
        self._awaiting_enable_ack.clear()
        self.inhibited_since.update({joint: time.monotonic() for joint in self.bus_by_joint})
        with self.active_command_lock:
            self.active_commands.clear()

        all_disabled = True
        for joint_name, bus in self.bus_by_joint.items():
            motor_name = self.motor_name_by_joint[joint_name]
            try:
                with self.bus_locks[bus.channel]:
                    bus.disable(motor_name)
                    self.current_mode_by_joint[joint_name] = None
            except Exception as exc:
                all_disabled = False
                self.get_logger().error(
                    f'ESTOP disable failed for {joint_name}; retrying: {exc}'
                )
        if all_disabled:
            self._estop_disable_pending.clear()
            self.get_logger().fatal('ESTOP disable requests submitted; verify physical stop via feedback')

    def _drain_command_queue(self):
        # Bound work even when producers continuously update targets.
        for _ in range(2 * len(self.motor_name_by_joint)):
            self._expire_commands()
            try:
                cmd = self.command_queue.get_nowait()
            except queue.Empty:
                return

            joint_name = cmd['joint']
            if joint_name not in self.motor_name_by_joint:
                self.get_logger().warn(f'Unknown joint: {joint_name}')
                continue

            mode = cmd['mode']
            bus = self.bus_by_joint.get(joint_name)
            if bus is None:
                self.get_logger().warn(f'No bus for joint: {joint_name}')
                continue

            # Queue residence consumes the original lifetime. A delayed ENABLE
            # must not restart a joint after the watchdog has already stopped it.
            if mode != MotorCommand.MODE_DISABLE:
                if self.estop_active.is_set() or self.transport_fault.is_set() or time.monotonic() >= cmd['deadline']:
                    continue
                if mode == MotorCommand.MODE_ENABLE:
                    if joint_name in self.pending_disable:
                        continue
                    if cmd['received_at'] <= self.inhibited_since.get(joint_name, float('-inf')):
                        continue
                elif joint_name in self.inhibited_since and joint_name not in self.enable_pending:
                    continue

            try:
                if mode == MotorCommand.MODE_ENABLE:
                    # Arm before I/O: an enable may reach hardware even when
                    # its acknowledgement fails. Never retain an older target.
                    with self.active_command_lock:
                        self.active_commands.pop(joint_name, None)
                    self.command_deadlines[joint_name] = cmd['deadline']
                    self.enable_pending[joint_name] = time.monotonic()
                    self.inhibited_since.setdefault(joint_name, cmd['received_at'])
                    self._awaiting_enable_ack.set()
                    with self.bus_locks[bus.channel]:
                        motor_name = self.motor_name_by_joint[joint_name]
                        self._set_mode_raw(bus, motor_name, 0)
                        bus.enable(motor_name)
                        self.current_mode_by_joint[joint_name] = None
                    self.pending_disable.discard(joint_name)
                    self.command_deadlines[joint_name] = cmd['deadline']
                    self.get_logger().info(f'Enable submitted for {joint_name}')
                    continue

                if mode == MotorCommand.MODE_DISABLE:
                    with self.active_command_lock:
                        self.active_commands.pop(joint_name, None)
                    self.command_deadlines.pop(joint_name, None)
                    self.enable_pending.pop(joint_name, None)
                    self.inhibited_since[joint_name] = time.monotonic()
                    self.pending_disable.add(joint_name)
                    with self.bus_locks[bus.channel]:
                        bus.disable(self.motor_name_by_joint[joint_name])
                        self.current_mode_by_joint[joint_name] = None
                    self.pending_disable.discard(joint_name)
                    self.get_logger().info(f'Disable submitted for {joint_name}')
                    continue

                with self.active_command_lock:
                    previous_cmd = self.active_commands.get(joint_name)
                    self.active_commands[joint_name] = cmd
                if joint_name not in self.enable_pending:
                    self.command_deadlines[joint_name] = cmd['deadline']

                if mode == MotorCommand.MODE_VELOCITY and (
                    previous_cmd is None
                    or previous_cmd.get('mode') != MotorCommand.MODE_VELOCITY
                    or previous_cmd.get('acceleration') != cmd.get('acceleration')
                ):
                    self.current_mode_by_joint[joint_name] = None

                self.get_logger().debug(
                    f"Active command for {joint_name}: mode={mode}, "
                    f"pos={cmd['position']:.4f}, vel={cmd['velocity']:.4f}, "
                    f"kp={cmd['kp']:.2f}, kd={cmd['kd']:.2f}"
                )

            except Exception as e:
                if mode == MotorCommand.MODE_ENABLE:
                    self.command_deadlines.pop(joint_name, None)
                    self.enable_pending.pop(joint_name, None)
                    self.inhibited_since[joint_name] = time.monotonic()
                    self.pending_disable.add(joint_name)
                self.get_logger().error(
                    f'Failed to apply command for {joint_name}: {e}'
                )

    def _expire_commands(self):
        """Stop expired joints and retry failed disables on every worker pass.

        All deadlines use host monotonic seconds, including time spent queued or
        descheduled. A fresh explicit ENABLE is required to recover;
        ordinary motion traffic cannot renew a joint that has timed out.
        """
        now = time.monotonic()
        for joint, deadline in list(self.command_deadlines.items()):
            if now >= deadline:
                self.command_deadlines.pop(joint, None)
                self.enable_pending.pop(joint, None)
                self.inhibited_since[joint] = now
                with self.active_command_lock:
                    self.active_commands.pop(joint, None)
                self.pending_disable.add(joint)
                self.get_logger().error(f'Command timeout for {joint}; explicit enable required')
        if not self.enable_pending:
            self._awaiting_enable_ack.clear()
        for joint in list(self.pending_disable):
            bus = self.bus_by_joint[joint]
            try:
                with self.bus_locks[bus.channel]:
                    bus.disable(self.motor_name_by_joint[joint])
                    self.current_mode_by_joint[joint] = None
                self.pending_disable.discard(joint)
            except Exception as exc:
                self.get_logger().error(f'Watchdog disable failed for {joint}; retrying: {exc}')

    def _send_active_command(self, cmd):
        # Host scheduling may exhaust the deadline after the active snapshot.
        self._expire_commands()
        joint_name = cmd['joint']
        if self.estop_active.is_set() or self.transport_fault.is_set() or joint_name in self.inhibited_since:
            return
        bus = self.bus_by_joint.get(joint_name)
        motor_name = self.motor_name_by_joint.get(joint_name)
        if bus is None or motor_name is None:
            return

        mode = cmd['mode']

        try:
            cmd = self.motor_settings[joint_name].command(cmd)
            with self.bus_locks[bus.channel]:
                if mode == MotorCommand.MODE_VELOCITY:
                    self._send_velocity_command(bus, joint_name, motor_name, cmd)
                    return
                elif mode == MotorCommand.MODE_POSITION:
                    self._send_position_command(bus, joint_name, motor_name, cmd)
                elif mode == MotorCommand.MODE_MOTION:
                    self._send_motion_command(bus, joint_name, motor_name, cmd)
        except Exception as e:
            self.timing['tx_errors'] += 1
            self.get_logger().error(
                f'Failed to send active command to {joint_name}: {e}', throttle_duration_sec=1.0
            )

    def _send_velocity_command(self, bus: RobstrideBus, joint_name: str, motor_name: str, cmd):
        if self.current_mode_by_joint.get(joint_name) != 2:
            self._write_parameter(
                bus,
                joint_name,
                motor_name,
                ParameterType.MODE,
                2,
            )
            self._write_parameter(
                bus,
                joint_name,
                motor_name,
                ParameterType.CURRENT_LIMIT,
                5.0,
            )
            self._write_parameter(
                bus,
                joint_name,
                motor_name,
                ParameterType.VEL_ACCELERATION_TARGET,
                cmd.get('acceleration', 10.0),
            )
            self._write_parameter(
                bus,
                joint_name,
                motor_name,
                ParameterType.VELOCITY_TARGET,
                0.0,
            )
            self.current_mode_by_joint[joint_name] = 2

        vel_cmd = self._clamp(
            cmd['velocity'],
            -self.max_vel_rad_s,
            self.max_vel_rad_s,
        )
        self._write_parameter(
            bus,
            joint_name,
            motor_name,
            ParameterType.VELOCITY_TARGET,
            vel_cmd,
        )

    def _send_position_command(self, bus: RobstrideBus, joint_name: str, motor_name: str, cmd):
        if self.current_mode_by_joint.get(joint_name) != 0:
            self._set_mode_raw(bus, motor_name, 0)
            self.current_mode_by_joint[joint_name] = 0

        vel_cmd = self._clamp(
            cmd.get('velocity', 0.0),
            -self.max_vel_rad_s,
            self.max_vel_rad_s,
        )
        bus.write_operation_frame(
            motor_name,
            cmd['position'],
            cmd.get('kp', self.default_kp),
            cmd.get('kd', self.default_kd),
            abs(vel_cmd),
            0.0,
        )

    def _send_motion_command(self, bus: RobstrideBus, joint_name: str, motor_name: str, cmd):
        if self.current_mode_by_joint.get(joint_name) != 0:
            self._set_mode_raw(bus, motor_name, 0)
            self.current_mode_by_joint[joint_name] = 0

        vel_cmd = self._clamp(
            cmd['velocity'],
            -self.max_vel_rad_s,
            self.max_vel_rad_s,
        )
        bus.write_operation_frame(
            motor_name,
            cmd['position'],
            cmd.get('kp', self.default_kp),
            cmd.get('kd', self.default_kd),
            vel_cmd,
            cmd.get('torque', 0.0),
        )

    def _safety_estop_callback(self, msg: Bool):
        if msg.data:
            newly_asserted = not self.estop_active.is_set()
            self.estop_active.set()
            self._estop_disable_pending.set()
            self._tx_wakeup.set()
            with self.active_command_lock:
                self.active_commands.clear()
            while True:
                try:
                    self.command_queue.get_nowait()
                except queue.Empty:
                    break
            if newly_asserted:
                self.get_logger().fatal(
                    'Safety estop asserted; rejecting commands and disabling motors'
                )
        else:
            was_active = self.estop_active.is_set()
            self.estop_active.clear()
            if was_active:
                self.get_logger().warn(
                    'Safety estop reset; motors remain disabled until explicitly enabled'
                )

    def _cmd_callback(self, msg: MotorCommand):
        if self.estop_active.is_set() or self.transport_fault.is_set():
            return
        if not msg.joint_name:
            return

        received_at = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now_ros - stamp
        source_valid = (msg.header.stamp.sec >= 0 and msg.header.stamp.nanosec < 10**9
                        and stamp > 0 and -0.01 <= age < self.command_timeout_s)
        # The source age and local queue time both count against the lifetime.
        deadline = received_at + self.command_timeout_s - max(0.0, age)
        expiry = msg.valid_until.sec + msg.valid_until.nanosec * 1e-9
        if expiry != 0:
            source_valid = (source_valid and msg.valid_until.sec >= 0
                            and msg.valid_until.nanosec < 10**9
                            and expiry > now_ros and expiry > stamp)
            deadline = min(deadline, received_at + expiry - now_ros)

        if len(msg.mode) == 1:
            modes = [msg.mode[0]] * len(msg.joint_name)
        elif len(msg.mode) == 0:
            modes = [MotorCommand.MODE_VELOCITY] * len(msg.joint_name)
        else:
            modes = msg.mode

        commands = []
        for i, joint in enumerate(msg.joint_name):
            if joint not in self.motor_name_by_joint:
                continue

            mode = modes[i] if i < len(modes) else MotorCommand.MODE_VELOCITY
            if mode != MotorCommand.MODE_DISABLE and not source_valid:
                self.get_logger().warn(f'Rejecting stale or invalid command timestamp for {joint}')
                continue

            cmd = {
                'joint': joint,
                'mode': mode,
                'received_at': received_at,
                'deadline': deadline,
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'acceleration': msg.acceleration[i] if i < len(msg.acceleration) else 0.0,
                'torque': msg.torque[i] if i < len(msg.torque) else 0.0,
                'kp': msg.kp[i] if i < len(msg.kp) else self.motor_settings[joint].kp,
                'kd': msg.kd[i] if i < len(msg.kd) else self.motor_settings[joint].kd,
            }

            commands.append(cmd)
        if commands:
            self.command_queue.put_nowait(commands)
            self._tx_wakeup.set()

    def _publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        oldest_age = 0.0

        with self.state_lock:
            now = time.monotonic()
            for joint_name in self.motor_name_by_joint:
                sample = self.state_buffer.get(joint_name)
                if sample is None or now - sample[4] > self.feedback_timeout_s:
                    continue
                pos, vel, tq, temp, timestamp = sample
                oldest_age = max(oldest_age, now - timestamp)
                msg.name.append(joint_name)
                msg.position.append(pos)
                msg.velocity.append(vel)
                msg.effort.append(tq)

        if msg.name:
            # JointState has only one stamp: conservatively use the oldest
            # included host receipt, rather than making cached data look new.
            stamp_ns = max(0, msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                           - int(oldest_age * 10**9))
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
            self.joint_state_pub.publish(msg)

        self._publish_motor_status()

    def _publish_motor_status(self):
        status_by_index = {}
        telemetry = []

        with self.state_lock:
            for joint_name, (pos, vel, tq, temp, timestamp) in self.state_buffer.items():
                if time.monotonic() - timestamp > self.feedback_timeout_s:
                    continue
                telemetry.append(dict(
                    self.motor_identity[joint_name],
                    source_stamp=self.telemetry_stamps[joint_name],
                    temperature_c=float(temp)))
                motor_index = self.motor_index_map.get(joint_name)
                if motor_index is None:
                    continue
                status_by_index[str(motor_index)] = {
                    'temperature': float(temp),
                    'torque': float(tq),
                }

        msg = String()
        msg.data = json.dumps(status_by_index)
        self.motor_status_pub.publish(msg)
        stamped = String()
        stamped.data = json.dumps(dict(schema_version=1, motors=telemetry), allow_nan=False)
        self.telemetry_pub.publish(stamped)

    def destroy_node(self):
        self.get_logger().info('Shutting down Python CAN node...')
        self.running = False
        self._tx_wakeup.set()
        # Join owners before sending final disables and closing their sockets.
        for thread in [self.can_tx_thread] + self.can_rx_threads:
            thread.join(timeout=1.0)
            if thread.is_alive():
                raise RuntimeError(f'{thread.name} did not stop; CAN sockets left open')

        for joint_name, bus in self.bus_by_joint.items():
            motor_name = self.motor_name_by_joint[joint_name]
            try:
                with self.bus_locks[bus.channel]:
                    bus.disable(motor_name)
            except Exception:
                pass

        for bus in self.buses.values():
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PythonCanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
