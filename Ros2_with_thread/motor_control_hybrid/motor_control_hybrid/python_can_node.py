#!/usr/bin/env python3
"""
ROS2 CAN node using the installed robstride_dynamics SDK directly.

The CAN access pattern intentionally matches the working scripts in
robstride_control/MotorTest:
- one RobstrideBus per CAN interface
- explicit enable/disable only
- continuous command loop for active commands
- feedback is read by the same thread that writes commands
"""

import json
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
from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PythonCanNode(Node):
    def __init__(self):
        super().__init__('python_can_node')

        self.declare_parameter('motor_config_file', '')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('feedback_poll_hz', 50.0)
        self.declare_parameter('feedback_poll_when_idle', True)

        cfg_path = self.get_parameter('motor_config_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        feedback_poll_hz = self.get_parameter('feedback_poll_hz').get_parameter_value().double_value
        self.feedback_poll_when_idle = (
            self.get_parameter('feedback_poll_when_idle').get_parameter_value().bool_value
        )

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

        self._load_config_and_connect(cfg_path)

        self.state_buffer: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.state_lock = threading.Lock()

        self.command_queue = queue.Queue()
        self.active_commands = {}
        self.active_command_lock = threading.Lock()
        self.tx_period_s = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.feedback_poll_period_s = (
            1.0 / feedback_poll_hz if feedback_poll_hz > 0.0 else self.tx_period_s
        )
        self._next_feedback_poll_time = 0.0
        self.running = True

        self.can_tx_thread = threading.Thread(
            target=self._can_tx_loop,
            daemon=True,
            name='CAN_TX_Thread',
        )
        self.can_tx_thread.start()

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_status_pub = self.create_publisher(String, 'motor_status', 10)
        self.cmd_sub = self.create_subscription(
            MotorCommand,
            'motor_commands',
            self._cmd_callback,
            10,
        )

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self._publish_states)

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

        motors_cfg = params.get('motors', {}) or {}
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
                bus = RobstrideBus(iface, motors, {})
                bus.connect(handshake=True)
                self.buses[iface] = bus
                self.bus_locks[iface] = threading.Lock()
                for joint_name, motor_name in self.motor_name_by_joint.items():
                    if motor_name in motors:
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

    def _write_parameter_and_update_feedback(
        self,
        bus: RobstrideBus,
        joint_name: str,
        motor_name: str,
        parameter_type: tuple,
        value,
        timeout: float = 0.1,
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

        try:
            pos, vel, tq, temp = bus.receive_status_frame(motor_name)
            self._update_state(joint_name, pos, vel, tq, temp)
            return True
        except Exception as e:
            if 'No response' not in str(e):
                self.get_logger().warn(
                    f'Write feedback failed for {joint_name} ({param_name}): {e}'
                )
            return False

    def _update_state(self, joint_name: str, pos: float, vel: float, tq: float, temp: float):
        with self.state_lock:
            self.state_buffer[joint_name] = (pos, vel, tq, temp, time.time())

    def _try_read_feedback(self, joint_name: str):
        bus = self.bus_by_joint.get(joint_name)
        motor_name = self.motor_name_by_joint.get(joint_name)
        if bus is None or motor_name is None:
            return

        try:
            pos, vel, tq, temp = bus.read_operation_frame(motor_name)
            self._update_state(joint_name, pos, vel, tq, temp)
        except Exception as e:
            if 'No response' not in str(e):
                self.get_logger().warn(f'Feedback read failed for {joint_name}: {e}')

    def _can_tx_loop(self):
        self.get_logger().info('CAN TX/control thread started')

        while self.running and rclpy.ok():
            self._drain_command_queue()

            with self.active_command_lock:
                active = list(self.active_commands.values())

            for cmd in active:
                self._send_active_command(cmd)

            if self.feedback_poll_when_idle:
                self._poll_all_feedback_if_due()

            time.sleep(self.tx_period_s)

    def _poll_all_feedback_if_due(self):
        now = time.time()
        if now < self._next_feedback_poll_time:
            return
        self._next_feedback_poll_time = now + self.feedback_poll_period_s

        active_joints = set()
        with self.active_command_lock:
            active_joints.update(self.active_commands.keys())

        for joint_name in self.motor_name_by_joint.keys():
            if joint_name in active_joints:
                continue

            bus = self.bus_by_joint.get(joint_name)
            if bus is None:
                continue

            try:
                with self.bus_locks[bus.channel]:
                    self._try_read_feedback(joint_name)
            except Exception as e:
                if 'No response' not in str(e):
                    self.get_logger().warn(
                        f'Idle feedback poll failed for {joint_name}: {e}'
                    )

    def _drain_command_queue(self):
        while True:
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

            try:
                if mode == MotorCommand.MODE_ENABLE:
                    with self.bus_locks[bus.channel]:
                        motor_name = self.motor_name_by_joint[joint_name]
                        bus.enable(motor_name)
                        self._set_mode_raw(bus, motor_name, 0)
                        self.current_mode_by_joint[joint_name] = None
                        time.sleep(0.05)
                        self._try_read_feedback(joint_name)
                    self.get_logger().info(f'Enabled {joint_name}')
                    continue

                if mode == MotorCommand.MODE_DISABLE:
                    with self.active_command_lock:
                        self.active_commands.pop(joint_name, None)
                    with self.bus_locks[bus.channel]:
                        bus.disable(self.motor_name_by_joint[joint_name])
                        self.current_mode_by_joint[joint_name] = None
                    self.get_logger().info(f'Disabled {joint_name}')
                    continue

                with self.active_command_lock:
                    previous_cmd = self.active_commands.get(joint_name)
                    self.active_commands[joint_name] = cmd

                if mode == MotorCommand.MODE_VELOCITY and (
                    previous_cmd is None
                    or previous_cmd.get('mode') != MotorCommand.MODE_VELOCITY
                    or previous_cmd.get('acceleration') != cmd.get('acceleration')
                ):
                    self.current_mode_by_joint[joint_name] = None

                self.get_logger().info(
                    f"Active command for {joint_name}: mode={mode}, "
                    f"pos={cmd['position']:.4f}, vel={cmd['velocity']:.4f}, "
                    f"kp={cmd['kp']:.2f}, kd={cmd['kd']:.2f}"
                )

            except Exception as e:
                self.get_logger().error(
                    f'Failed to apply command for {joint_name}: {e}'
                )

    def _send_active_command(self, cmd):
        joint_name = cmd['joint']
        bus = self.bus_by_joint.get(joint_name)
        motor_name = self.motor_name_by_joint.get(joint_name)
        if bus is None or motor_name is None:
            return

        mode = cmd['mode']

        try:
            with self.bus_locks[bus.channel]:
                if mode == MotorCommand.MODE_VELOCITY:
                    self._send_velocity_command(bus, joint_name, motor_name, cmd)
                    return
                elif mode == MotorCommand.MODE_POSITION:
                    self._send_position_command(bus, joint_name, motor_name, cmd)
                elif mode == MotorCommand.MODE_MOTION:
                    self._send_motion_command(bus, joint_name, motor_name, cmd)
                self._try_read_feedback(joint_name)
        except Exception as e:
            self.get_logger().error(
                f'Failed to send active command to {joint_name}: {e}'
            )

    def _send_velocity_command(self, bus: RobstrideBus, joint_name: str, motor_name: str, cmd):
        if self.current_mode_by_joint.get(joint_name) != 2:
            self._write_parameter_and_update_feedback(
                bus,
                joint_name,
                motor_name,
                ParameterType.MODE,
                2,
            )
            self._write_parameter_and_update_feedback(
                bus,
                joint_name,
                motor_name,
                ParameterType.CURRENT_LIMIT,
                5.0,
            )
            self._write_parameter_and_update_feedback(
                bus,
                joint_name,
                motor_name,
                ParameterType.VEL_ACCELERATION_TARGET,
                cmd.get('acceleration', 10.0),
            )
            self._write_parameter_and_update_feedback(
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
        self._write_parameter_and_update_feedback(
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

    def _cmd_callback(self, msg: MotorCommand):
        if not msg.joint_name:
            return

        if len(msg.mode) == 1:
            modes = [msg.mode[0]] * len(msg.joint_name)
        elif len(msg.mode) == 0:
            modes = [MotorCommand.MODE_VELOCITY] * len(msg.joint_name)
        else:
            modes = msg.mode

        for i, joint in enumerate(msg.joint_name):
            if joint not in self.motor_name_by_joint:
                continue

            cmd = {
                'joint': joint,
                'mode': modes[i] if i < len(modes) else MotorCommand.MODE_VELOCITY,
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'acceleration': msg.acceleration[i] if i < len(msg.acceleration) else 0.0,
                'torque': msg.torque[i] if i < len(msg.torque) else 0.0,
                'kp': msg.kp[i] if i < len(msg.kp) else self.default_kp,
                'kd': msg.kd[i] if i < len(msg.kd) else self.default_kd,
            }

            try:
                self.command_queue.put_nowait(cmd)
            except queue.Full:
                self.get_logger().warn(f'Command queue full, dropping command for {joint}')

    def _publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        with self.state_lock:
            for joint_name, (pos, vel, tq, temp, timestamp) in self.state_buffer.items():
                msg.name.append(joint_name)
                msg.position.append(pos)
                msg.velocity.append(vel)
                msg.effort.append(tq)

        if msg.name:
            self.joint_state_pub.publish(msg)

        self._publish_motor_status()

    def _publish_motor_status(self):
        status_by_index = {}

        with self.state_lock:
            for joint_name, (pos, vel, tq, temp, timestamp) in self.state_buffer.items():
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

    def destroy_node(self):
        self.get_logger().info('Shutting down Python CAN node...')
        self.running = False

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
