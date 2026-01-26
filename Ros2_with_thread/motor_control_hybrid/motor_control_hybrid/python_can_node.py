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

import rclpy
from rclpy.node import Node
import threading
import queue
import time
import yaml
import os
from typing import Dict, Tuple, Optional

from sensor_msgs.msg import JointState
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
        
        # Parameters
        self.declare_parameter('motor_config_file', '')
        self.declare_parameter('publish_rate_hz', 50.0)
        
        cfg_path = self.get_parameter('motor_config_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        
        # Load motor configuration
        self.drivers: Dict[str, RobStrideMotorLinux] = {}
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
            motors_cfg = params.get('motors', {})
            
            # Create motor drivers
            for joint_name, cfg in motors_cfg.items():
                iface = cfg.get('can_interface', self.default_iface)
                master_id = cfg.get('master_id', self.default_master)
                motor_id = cfg['motor_id']
                actuator_type = cfg.get('actuator_type', 0)
                
                self.get_logger().info(
                    f'Initializing motor: {joint_name} on {iface}, '
                    f'master={master_id}, motor_id={motor_id}'
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
        
        # Shared state buffer (thread-safe)
        self.state_buffer: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.state_lock = threading.Lock()
        
        # Command queue (thread-safe)
        self.command_queue = queue.Queue()
        
        # Control flags
        self.running = True
        
        # CAN RX thread (high priority, continuous receive)
        self.can_rx_thread = threading.Thread(
            target=self._can_rx_loop,
            daemon=True,
            name='CAN_RX_Thread'
        )
        self.can_rx_thread.start()
        
        # CAN TX thread (reads from queue and sends)
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
        
        self.cmd_sub = self.create_subscription(
            MotorCommand, 'motor_commands',
            self._cmd_callback, 10
        )
        
        # Timer for publishing states (50Hz)
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._publish_states)
        
        self.get_logger().info(
            f'Python CAN node started with {len(self.drivers)} motors'
        )
    
    def _can_rx_loop(self):
        """
        Independent thread: Continuously receives CAN status frames.
        Updates shared state buffer without blocking ROS2 callbacks.
        """
        self.get_logger().info('CAN RX thread started')
        
        while self.running and rclpy.ok():
            for joint_name, motor in self.drivers.items():
                try:
                    # Non-blocking receive (timeout=0.01s)
                    pos, vel, tq, temp = motor.receive_status_frame(timeout=0.01)
                    
                    # Update shared buffer (thread-safe)
                    with self.state_lock:
                        self.state_buffer[joint_name] = (
                            pos, vel, tq, temp, time.time()
                        )
                except Exception as e:
                    # Silently continue on error (motor may not respond)
                    pass
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)  # 1ms
    
    def _can_tx_loop(self):
        """
        Independent thread: Reads commands from queue and sends to motors.
        """
        self.get_logger().info('CAN TX thread started')
        
        while self.running and rclpy.ok():
            try:
                # Get command from queue (timeout to allow checking running flag)
                cmd = self.command_queue.get(timeout=0.1)
                
                joint_name = cmd['joint']
                if joint_name not in self.drivers:
                    self.get_logger().warn(f'Unknown joint: {joint_name}')
                    continue
                
                motor = self.drivers[joint_name]
                mode = cmd['mode']
                
                try:
                    if mode == MotorCommand.MODE_VELOCITY:
                        motor.send_velocity_mode_command(
                            velocity_rad_s=cmd['velocity']
                        )
                    elif mode == MotorCommand.MODE_POSITION:
                        motor.pos_pp_control(
                            speed_rad_s=cmd.get('velocity', 0.0),
                            acceleration_rad_s2=cmd.get('acceleration', 0.0),
                            angle_rad=cmd['position']
                        )
                    elif mode == MotorCommand.MODE_MOTION:
                        motor.send_motion_command(
                            torque=cmd.get('torque', 0.0),
                            position_rad=cmd['position'],
                            velocity_rad_s=cmd['velocity'],
                            kp=cmd.get('kp', 40.0),
                            kd=cmd.get('kd', 1.5)
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
        
        # Determine mode: if size==1, broadcast; if empty, default to VELOCITY
        modes = []
        if len(msg.mode) == 1:
            # Broadcast mode to all joints
            modes = [msg.mode[0]] * len(msg.joint_name)
        elif len(msg.mode) == 0:
            # Default to VELOCITY mode
            modes = [MotorCommand.MODE_VELOCITY] * len(msg.joint_name)
        else:
            # Use mode array as-is (must match joint_name length)
            modes = msg.mode
        
        # Process each joint
        for i, joint in enumerate(msg.joint_name):
            if joint not in self.drivers:
                continue
            
            # Get mode for this joint (safe array access)
            mode = modes[i] if i < len(modes) else MotorCommand.MODE_VELOCITY
            
            # Build command dict with safe array access and defaults
            cmd = {
                'joint': joint,
                'mode': mode,
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'acceleration': msg.acceleration[i] if i < len(msg.acceleration) else 0.0,
                'torque': msg.torque[i] if i < len(msg.torque) else 0.0,
                'kp': msg.kp[i] if i < len(msg.kp) else 40.0,
                'kd': msg.kd[i] if i < len(msg.kd) else 1.5,
            }
            
            # Put command in queue (non-blocking if queue is full)
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
                # Temperature could be added as custom field if needed
        
        if msg.name:  # Only publish if we have data
            self.joint_state_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down Python CAN node...')
        self.running = False
        
        # Disable all motors
        for joint_name, motor in self.drivers.items():
            try:
                motor.disable_motor()
            except:
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