#!/bin/bash
# Test command for position control (position=0)
# This script publishes a MotorCommand message with MODE_POSITION to test position control

ros2 topic pub --once /motor_commands motor_control_interfaces/msg/MotorCommand "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_name: ['joint1']  # Replace with actual joint name
mode: [1]  # MODE_POSITION
position: [0.0]
velocity: [0.0]
acceleration: [0.0]
torque: [0.0]
kp: [40.0]
kd: [1.5]
"
