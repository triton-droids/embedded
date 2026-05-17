#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Swing Leg Node - Integrates swing leg program from robstride branch

This node implements sinusoidal knee sweep and coupled ankle motion:
- Knee motors (left_knee, right_knee) move sinusoidally between 0 rad and -1.57 rad
- They are opposite in phase (phase-shifted by pi)
- Ankle motors (left_ankle, right_ankle) are coupled to their respective knees
- Uses MIT mode (CMD_MOTION) via the ROS2 service interface

Based on leg_swing_test.py from the robstride_control branch.
"""

import math
import rclpy
from rclpy.node import Node
from motor_control_interfaces.srv import RobStrideJointControl


class SwingLegNode(Node):
    """
    ROS2 node that implements the swing leg motion pattern.
    
    Subscribes to: none
    Publishes: none
    Services: calls /robstride_joint_control for motor commands
    """

    def __init__(self):
        super().__init__('swing_leg_node')

        # Parameters
        self.declare_parameter('service_name', 'robstride_joint_control')
        self.declare_parameter('frequency_hz', 0.3)  # Swing frequency
        self.declare_parameter('control_rate_hz', 50.0)  # Control loop rate
        self.declare_parameter('max_velocity_rad_s', 1.8)  # Velocity limit
        self.declare_parameter('kp', 10.0)  # MIT control gain
        self.declare_parameter('kd', 0.2)  # MIT control gain
        self.declare_parameter('enable_on_start', True)  # Enable motors on startup

        self.service_name = self.get_parameter(
            'service_name').get_parameter_value().string_value
        self.freq_hz = self.get_parameter(
            'frequency_hz').get_parameter_value().double_value
        self.control_rate_hz = self.get_parameter(
            'control_rate_hz').get_parameter_value().double_value
        self.max_vel_rad_s = self.get_parameter(
            'max_velocity_rad_s').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.enable_on_start = self.get_parameter(
            'enable_on_start').get_parameter_value().bool_value

        # Service client
        self.client = self.create_client(RobStrideJointControl, self.service_name)
        
        # Joint names (matching config file)
        self.JOINT_LEFT_KNEE = 'left_knee'
        self.JOINT_LEFT_ANKLE = 'left_ankle'
        self.JOINT_RIGHT_KNEE = 'right_knee'
        self.JOINT_RIGHT_ANKLE = 'right_ankle'
        
        self.joints = [
            self.JOINT_LEFT_KNEE,
            self.JOINT_LEFT_ANKLE,
            self.JOINT_RIGHT_KNEE,
            self.JOINT_RIGHT_ANKLE
        ]

        # Direction signs (from original script: left side +1, right side -1)
        # Note: The original script used these to convert logical to physical coordinates
        # Since the service interface may expect physical coordinates, we apply these
        self.direction_signs = {
            self.JOINT_LEFT_KNEE: +1,
            self.JOINT_LEFT_ANKLE: +1,
            self.JOINT_RIGHT_KNEE: -1,
            self.JOINT_RIGHT_ANKLE: -1,
        }

        # Knee logical limits (radians)
        self.KNEE_LO = -1.57  # -90 degrees
        self.KNEE_HI = 0.0
        
        # Ankle logical limits (radians): [-30deg, 0deg]
        self.ANKLE_LO = -math.radians(30.0)  # -0.523599...
        self.ANKLE_HI = 0.0

        # Sine parameters
        self.knee_center = 0.5 * (self.KNEE_LO + self.KNEE_HI)  # -0.785 rad
        self.knee_amp = 0.5 * (self.KNEE_HI - self.KNEE_LO)  # 0.785 rad

        # Control loop timing
        self.dt = 1.0 / self.control_rate_hz
        self.max_step = self.max_vel_rad_s * self.dt

        # State tracking (logical positions)
        self.state = {
            joint: {'cmd_logical': 0.0} for joint in self.joints
        }
        
        # Timing
        self.start_time = None
        self.running = False

        self.get_logger().info(
            f'Swing leg node initializing. Waiting for service {self.service_name}...'
        )

    def wait_for_service(self):
        """Wait for the motor control service to be available."""
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for service {self.service_name}...')

    def call_service_sync(self, joint_name: str, command_type: int, **kwargs):
        """
        Call the motor control service synchronously.
        
        Args:
            joint_name: Name of the joint
            command_type: Command type (CMD_ENABLE, CMD_MOTION, etc.)
            **kwargs: Additional parameters (position, velocity, kp, kd, etc.)
        
        Returns:
            Response from the service, or None if failed
        """
        req = RobStrideJointControl.Request()
        req.joint_name = joint_name
        req.command_type = command_type
        req.position = kwargs.get('position', 0.0)
        req.velocity = kwargs.get('velocity', 0.0)
        req.torque = kwargs.get('torque', 0.0)
        req.acceleration = kwargs.get('acceleration', 0.0)
        req.kp = kwargs.get('kp', 0.0)
        req.kd = kwargs.get('kd', 0.0)
        req.iq = kwargs.get('iq', 0.0)
        req.id = kwargs.get('id', 0.0)

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Service call failed for {joint_name}')
            return None

    def enable_motors(self):
        """Enable all motors."""
        self.get_logger().info('Enabling motors...')
        for joint in self.joints:
            resp = self.call_service_sync(joint, RobStrideJointControl.Request.CMD_ENABLE)
            if resp and resp.success:
                self.get_logger().info(f'{joint}: enabled')
            else:
                self.get_logger().warn(f'{joint}: enable failed')
        
        # Small delay after enabling
        rclpy.spin_once(self, timeout_sec=0.2)

    def disable_motors(self):
        """Disable all motors."""
        self.get_logger().info('Disabling motors...')
        for joint in self.joints:
            try:
                resp = self.call_service_sync(joint, RobStrideJointControl.Request.CMD_DISABLE)
                if resp and resp.success:
                    self.get_logger().info(f'{joint}: disabled')
            except Exception as e:
                self.get_logger().error(f'Error disabling {joint}: {e}')

    def clamp(self, x: float, lo: float, hi: float) -> float:
        """Clamp value between limits."""
        return max(lo, min(hi, x))

    def knee_to_ankle(self, knee_rad: float) -> float:
        """
        Map knee position to ankle position.
        
        Maps knee in [0, -1.57] to ankle in [-30deg, 0] such that:
          knee = 0      -> ankle = -30deg
          knee = -1.57  -> ankle = 0
        """
        knee_rad = self.clamp(knee_rad, self.KNEE_LO, self.KNEE_HI)
        # alpha=0 at knee=0, alpha=1 at knee=-1.57
        alpha = (self.KNEE_HI - knee_rad) / (self.KNEE_HI - self.KNEE_LO)
        ankle = self.ANKLE_LO * (1.0 - alpha) + self.ANKLE_HI * alpha
        return self.clamp(ankle, self.ANKLE_LO, self.ANKLE_HI)

    def ramp_toward(self, cmd: float, target: float, max_step: float, lo: float, hi: float) -> float:
        """Ramp commanded position toward target with velocity limit."""
        delta = target - cmd
        if abs(delta) <= max_step:
            cmd = target
        else:
            cmd = cmd + math.copysign(max_step, delta)
        return self.clamp(cmd, lo, hi)

    def initialize_state(self):
        """
        Initialize state by reading current positions.
        This prevents sudden jumps when starting the motion.
        """
        self.get_logger().info('Reading current motor positions...')
        for joint in self.joints:
            resp = self.call_service_sync(joint, RobStrideJointControl.Request.CMD_READ_STATUS)
            if resp and resp.success:
                # Convert physical position to logical (divide by direction sign)
                logical_pos = resp.position / float(self.direction_signs[joint])
                self.state[joint]['cmd_logical'] = logical_pos
                # Send a hold command at current position
                physical_pos = logical_pos * float(self.direction_signs[joint])
                self.call_service_sync(
                    joint,
                    RobStrideJointControl.Request.CMD_MOTION,
                    position=physical_pos,
                    velocity=0.0,
                    kp=self.kp,
                    kd=self.kd
                )
            else:
                self.get_logger().warn(f'Failed to read position for {joint}, using 0.0')
                self.state[joint]['cmd_logical'] = 0.0
        
        # Small delay after initialization
        rclpy.spin_once(self, timeout_sec=0.1)

    def control_loop(self):
        """Main control loop called by timer."""
        if not self.running:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Calculate elapsed time
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time

        # Calculate sinusoidal targets
        s = math.sin(2.0 * math.pi * self.freq_hz * t)

        # Knees are opposite in phase
        target_left_knee = self.clamp(
            self.knee_center + self.knee_amp * s,
            self.KNEE_LO, self.KNEE_HI
        )
        target_right_knee = self.clamp(
            self.knee_center - self.knee_amp * s,
            self.KNEE_LO, self.KNEE_HI
        )

        # Ankles are coupled to their respective knees
        target_left_ankle = self.knee_to_ankle(target_left_knee)
        target_right_ankle = self.knee_to_ankle(target_right_knee)

        # Ramp each motor toward its target (with velocity limit)
        self.state[self.JOINT_LEFT_KNEE]['cmd_logical'] = self.ramp_toward(
            self.state[self.JOINT_LEFT_KNEE]['cmd_logical'],
            target_left_knee,
            self.max_step,
            self.KNEE_LO,
            self.KNEE_HI
        )
        self.state[self.JOINT_RIGHT_KNEE]['cmd_logical'] = self.ramp_toward(
            self.state[self.JOINT_RIGHT_KNEE]['cmd_logical'],
            target_right_knee,
            self.max_step,
            self.KNEE_LO,
            self.KNEE_HI
        )
        self.state[self.JOINT_LEFT_ANKLE]['cmd_logical'] = self.ramp_toward(
            self.state[self.JOINT_LEFT_ANKLE]['cmd_logical'],
            target_left_ankle,
            self.max_step,
            self.ANKLE_LO,
            self.ANKLE_HI
        )
        self.state[self.JOINT_RIGHT_ANKLE]['cmd_logical'] = self.ramp_toward(
            self.state[self.JOINT_RIGHT_ANKLE]['cmd_logical'],
            target_right_ankle,
            self.max_step,
            self.ANKLE_LO,
            self.ANKLE_HI
        )

        # Convert logical positions to physical and send commands
        # Note: Using async calls for better performance
        joints_to_control = [
            (self.JOINT_LEFT_KNEE, self.state[self.JOINT_LEFT_KNEE]['cmd_logical']),
            (self.JOINT_LEFT_ANKLE, self.state[self.JOINT_LEFT_ANKLE]['cmd_logical']),
            (self.JOINT_RIGHT_KNEE, self.state[self.JOINT_RIGHT_KNEE]['cmd_logical']),
            (self.JOINT_RIGHT_ANKLE, self.state[self.JOINT_RIGHT_ANKLE]['cmd_logical']),
        ]

        for joint_name, logical_pos in joints_to_control:
            physical_pos = logical_pos * float(self.direction_signs[joint_name])
            req = RobStrideJointControl.Request()
            req.joint_name = joint_name
            req.command_type = RobStrideJointControl.Request.CMD_MOTION
            req.position = physical_pos
            req.velocity = 0.0
            req.kp = self.kp
            req.kd = self.kd
            self.client.call_async(req)

    def start(self):
        """Start the swing leg motion."""
        self.get_logger().info('Starting swing leg motion...')
        self.get_logger().info(
            f'  Knee range: [{self.KNEE_LO:.3f}, {self.KNEE_HI:.3f}] rad ([-90deg, 0deg])'
        )
        self.get_logger().info(
            f'  Ankle range: [{self.ANKLE_LO:.3f}, {self.ANKLE_HI:.3f}] rad ([-30deg, 0deg])'
        )
        self.get_logger().info(f'  Frequency: {self.freq_hz:.3f} Hz (period {1.0/self.freq_hz:.2f} s)')
        self.get_logger().info(f'  Max velocity: {self.max_vel_rad_s:.2f} rad/s')
        self.get_logger().info(f'  Control rate: {self.control_rate_hz:.1f} Hz')
        
        if self.enable_on_start:
            self.enable_motors()
        
        self.initialize_state()
        
        # Create timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.running = True
        self.get_logger().info('Swing leg motion started. Press Ctrl+C to stop.')

    def stop(self):
        """Stop the swing leg motion."""
        self.running = False
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.disable_motors()
        self.get_logger().info('Swing leg motion stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = SwingLegNode()
    
    try:
        node.wait_for_service()
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
