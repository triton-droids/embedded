#!/usr/bin/env python3
"""
Python CAN Node with independent threads for real-time communication.

This node handles CAN communication for the hybrid control system.
It can run alongside the legacy debug node (motor_control_node_debug) which
provides service-based configuration commands.

Architecture:
- CAN RX thread: Continuously receives motor status frames, updates shared buffer
- CAN TX thread: Reads commands from queue and sends to motors
- ROS2 interface: Publishes joint_states and subscribes to joint_commands
"""

import json
import rclpy
from rclpy.node import Node
import threading
import queue
import time
import yaml
import os
from typing import Dict, Tuple

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from motor_control_interfaces.msg import MotorCommand

# Import motor driver from local copy
from motor_control_hybrid.robstride_motor_linux import RobStrideMotorLinux


class PythonCanNode(Node):
    """
    Python CAN node with independent threads for CAN communication.

    - CAN RX thread: High priority, continuously receives status frames
    - CAN TX thread: Reads commands from queue and sends
    - ROS2 interface: Low priority, publishes/subscribes topics
    """

    def __init__(self):
        super().__init__('python_can_node')

        # ROS parameters
        self.declare_parameter('motor_config_file', '')
        self.declare_parameter('publish_rate_hz', 50.0)

        cfg_path = self.get_parameter('motor_config_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Defaults in case YAML is missing
        self.default_iface = 'can0'
        self.default_master = 255
        self.default_kp = 10.0
        self.default_kd = 0.2
        self.max_vel_rad_s = 4.5
        self.freq_hz = 0.8

        # Load motor configuration
        self.drivers: Dict[str, RobStrideMotorLinux] = {}
        self.motor_index_map: Dict[str, int] = {}

        if cfg_path and os.path.exists(cfg_path):
            self.get_logger().info(f'Loading motor config from: {cfg_path}')
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f)

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

            motors_cfg = params.get('motors', {})

            self.get_logger().info(
                f'Global defaults loaded: '
                f'KP={self.default_kp}, KD={self.default_kd}, '
                f'MAX_VEL_RAD_S={self.max_vel_rad_s}, FREQ_HZ={self.freq_hz}'
            )

            # Create motor drivers
            for motor_index, (joint_name, cfg) in enumerate(motors_cfg.items()):
                iface = cfg.get('can_interface', self.default_iface)
                master_id = int(cfg.get('master_id', self.default_master))
                motor_id = int(cfg['motor_id'])
                actuator_type = int(cfg.get('actuator_type', 0))

                self.motor_index_map[joint_name] = motor_index

                self.get_logger().info(
                    f'Initializing motor[{motor_index}]: {joint_name} on {iface}, '
                    f'master={master_id}, motor_id={motor_id}, actuator_type={actuator_type}'
                )

                try:
                    self.drivers[joint_name] = RobStrideMotorLinux(
                        iface=iface,
                        master_id=master_id,
                        motor_id=motor_id,
                        actuator_type=actuator_type,
                    )
                except Exception as e:
                    self.get_logger().error(f'Failed to initialize {joint_name}: {e}')
        else:
            self.get_logger().warn(f'No motor config file found: {cfg_path}')
            self.get_logger().warn(
                f'Using built-in defaults: '
                f'KP={self.default_kp}, KD={self.default_kd}, '
                f'MAX_VEL_RAD_S={self.max_vel_rad_s}, FREQ_HZ={self.freq_hz}'
            )

        # Shared state buffer (thread-safe)
        self.state_buffer: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.state_lock = threading.Lock()

        # Command queue (thread-safe)
        self.command_queue = queue.Queue()

        # Control flags
        self.running = True

        # CAN RX thread
        self.can_rx_thread = threading.Thread(
            target=self._can_rx_loop,
            daemon=True,
            name='CAN_RX_Thread'
        )
        self.can_rx_thread.start()

        # CAN TX thread
        self.can_tx_thread = threading.Thread(
            target=self._can_tx_loop,
            daemon=True,
            name='CAN_TX_Thread'
        )
        self.can_tx_thread.start()

        # ROS2 publishers/subscribers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10
        )

        self.motor_status_pub = self.create_publisher(
            String, 'motor_status', 10
        )

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'motor_commands',
            self._cmd_callback, 10
        )

        # Timer for publishing states
        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self._publish_states)

        self.get_logger().info(
            f'Python CAN node started with {len(self.drivers)} motors'
        )

    def _clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def _can_rx_loop(self):
        """
        Independent thread: Continuously receives CAN status frames.
        Updates shared state buffer without blocking ROS2 callbacks.
        """
        self.get_logger().info('CAN RX thread started')

        while self.running and rclpy.ok():
            for joint_name, motor in self.drivers.items():
                try:
                    pos, vel, tq, temp = motor.receive_status_frame(timeout=0.01)

                    with self.state_lock:
                        self.state_buffer[joint_name] = (
                            pos, vel, tq, temp, time.time()
                        )
                except Exception:
                    pass

            time.sleep(0.001)

    def _can_tx_loop(self):
        """
        Independent thread: Reads commands from queue and sends to motors.
        """
        self.get_logger().info('CAN TX thread started')

        while self.running and rclpy.ok():
            try:
                cmd = self.command_queue.get(timeout=0.1)

                joint_name = cmd['joint']
                if joint_name not in self.drivers:
                    self.get_logger().warn(f'Unknown joint: {joint_name}')
                    continue

                motor = self.drivers[joint_name]
                mode = cmd['mode']

                try:
                    if mode == MotorCommand.MODE_VELOCITY:
                        vel_cmd = self._clamp(
                            cmd['velocity'],
                            -self.max_vel_rad_s,
                            self.max_vel_rad_s
                        )
                        motor.send_velocity_mode_command(
                            velocity_rad_s=vel_cmd
                        )

                    elif mode == MotorCommand.MODE_POSITION:
                        vel_cmd = self._clamp(
                            cmd.get('velocity', 0.0),
                            -self.max_vel_rad_s,
                            self.max_vel_rad_s
                        )
                        motor.pos_pp_control(
                            speed_rad_s=abs(vel_cmd),
                            acceleration_rad_s2=cmd.get('acceleration', 0.0),
                            angle_rad=cmd['position']
                        )

                    elif mode == MotorCommand.MODE_MOTION:
                        vel_cmd = self._clamp(
                            cmd['velocity'],
                            -self.max_vel_rad_s,
                            self.max_vel_rad_s
                        )
                        motor.send_motion_command(
                            torque=cmd.get('torque', 0.0),
                            position_rad=cmd['position'],
                            velocity_rad_s=vel_cmd,
                            kp=cmd.get('kp', self.default_kp),
                            kd=cmd.get('kd', self.default_kd)
                        )

                    elif mode == MotorCommand.MODE_ENABLE:
                        motor.enable_motor()

                    elif mode == MotorCommand.MODE_DISABLE:
                        motor.disable_motor()

                except Exception as e:
                    self.get_logger().error(
                        f'Failed to send command to {joint_name}: {e}'
                    )

            except queue.Empty:
                continue

    def _cmd_callback(self, msg: MotorCommand):
        """
        ROS2 callback: Receives commands and puts them in queue.
        Non-blocking, just enqueues commands.

        Parses joint_name[] and mode[]:
        - If mode size == 1, broadcast to all joints in joint_name[]
        - If mode is empty, default to MODE_VELOCITY
        """
        if not msg.joint_name:
            return

        # Determine mode
        if len(msg.mode) == 1:
            modes = [msg.mode[0]] * len(msg.joint_name)
        elif len(msg.mode) == 0:
            modes = [MotorCommand.MODE_VELOCITY] * len(msg.joint_name)
        else:
            modes = msg.mode

        for i, joint in enumerate(msg.joint_name):
            if joint not in self.drivers:
                continue

            mode = modes[i] if i < len(modes) else MotorCommand.MODE_VELOCITY

            cmd = {
                'joint': joint,
                'mode': mode,
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
        """
        ROS2 timer callback: Reads from shared buffer and publishes.
        Non-blocking, just reads latest state.
        """
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
        """Publish motor index -> {temperature, torque} as JSON on /motor_status."""
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
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down Python CAN node...')
        self.running = False

        for joint_name, motor in self.drivers.items():
            try:
                motor.disable_motor()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
