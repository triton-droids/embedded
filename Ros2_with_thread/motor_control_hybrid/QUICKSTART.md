# Quick Start Guide

## 1. Build the Package

```bash
cd /home/rcli/Github/embedded/embedded-humanoid-ros2/Ros2_with_thread
colcon build --packages-select motor_control_hybrid
source install/setup.bash
```

## 2. Configure CAN Interface (if not already configured)

```bash
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```

## 3. Edit Motor Configuration File

Edit `config/motors.yaml` to set your motor IDs and configuration.

## 4. Launch the System

```bash
ros2 launch motor_control_hybrid hybrid_control.launch.py
```

## 5. Monitor Data Flow

In another terminal:

```bash
# View joint states
ros2 topic echo /joint_states

# View joint commands
ros2 topic echo /joint_commands

# View node information
ros2 node list
ros2 node info /python_can_node
ros2 node info /cpp_control_node
```

## 6. Send Test Commands Manually

```bash
# Publish test command (zero velocity, safe)
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: ['shoulder_pitch']
position: [0.0]
velocity: [0.0]
effort: [0.0]
"
```

## Architecture Overview

- **Python CAN Node**: Handles all CAN communication using independent threads to avoid blocking
- **C++ Control Node**: Real-time control loop that can integrate RL policies
- **Communication**: Via ROS2 topics (`/joint_states`, `/joint_commands`)

## Next Steps

1. Implement your control logic in the C++ control node
2. Integrate RL model (ONNX Runtime)
3. Add safety checks and limits
4. Add other sensors (IMU, vision, etc.)
