# System Structure & Usage Guide

This repository is organized around a single principle:

> **ROS 2 is the single source of truth for sensing + broadcasting + CAN I/O.**  
> Anything outside ROS should consume data through a clearly defined interface, not by directly touching sensors.

---

## Architecture Diagram

![System Structure](system.struct.png)

### About the diagram file(s)

- `system.struct.png` is the **rendered image** used by this README.
- `System Structurepng.drawio` is the **editable source** (diagrams.net / draw.io).
---

## What the diagram means

### ROS 2 responsibilities (inside ROS)
ROS 2 owns:
- Reading sensors (IMU, camera, other drivers)
- Broadcasting all data through ROS topics (the “data bus”)
- Performing **motor CAN I/O** (send commands / receive motor states)

### Outside ROS responsibilities (external programs)
External programs should:
- **Not** access sensors directly
- Consume all data via an exported interface (e.g., a gateway API) or by linking a provided SDK
- Implement higher-level logic (logging, UI, analytics, ML, policies, etc.)

---

## Core components

### 1) Sensor drivers (ROS nodes)
Examples:
- IMU driver → publishes `/imu`
- Camera driver → publishes `/image_raw`
- Other sensors → publishes `/force`, `/encoders`, `/gps`, ...

### 2) Optional processing nodes (ROS nodes)
- Sensor fusion / estimator (optional) → publishes `/state_est`
- Perception / feature extraction (optional) → publishes `/features`

### 3) ROS Topics Broadcast Bus (data bus)
Typical topics:
- `/imu`, `/image_raw`, `/force`, `/encoders`, `/gps`, ...
- `/state_est` (optional), `/features` (optional)
- `/joint_states` (motor state from CAN)
- `/motor_commands` (motor commands to CAN)

### 4) Python CAN Node (ROS node, motor CAN I/O)
- **The only component that actually sends CAN frames**
- Publishes: `/joint_states` (`sensor_msgs/JointState`)
- Subscribes: `/motor_commands` (`motor_control_interfaces/MotorCommand`)
- Internals:
  - RX loop reads status frames and updates latest motor state
  - TX loop consumes a command queue and calls the motor driver to send CAN frames

### 5) Control node (ROS node, scheduler + bridge)
- Subscribes: `/joint_states` (+ optional `/imu`, `/features`, etc.)
- Produces motor commands and publishes `/motor_commands`
- Recommended: keep “control algorithms” in a **non-ROS control core library**, and make this node a thin adapter.

### 6) ROS-to-external interface (recommended for non-ROS clients)
If external programs truly must not depend on ROS, add a thin “exporter” process/node:
- subscribes to a whitelist of topics
- exposes:
  - `/stream` (WebSocket / gRPC / ZeroMQ)
  - `/latest` (HTTP / gRPC)
  - `/topics` (metadata)

---

## Interface contracts

### Motor state: `/joint_states`
Type: `sensor_msgs/JointState`

Index alignment rule:
- `name[i]` aligns with `position[i]`, `velocity[i]`, `effort[i]`

### Motor commands: `/motor_commands`
Type: `motor_control_interfaces/MotorCommand`

Index alignment rule:
- `joint_name[i]` aligns with:
  - `mode[i]`, `position[i]`, `velocity[i]`, `acceleration[i]`, `torque[i]`, `kp[i]`, `kd[i]`

Mode supports:
- **Broadcast**: `mode` length = 1 → applies to all joints in this message
- **Per-joint**: `mode` length = N → each joint can use a different mode

---

## Quick start (build & run)

### Build
In your ROS 2 workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_control_interfaces
source install/setup.bash
colcon build --packages-select motor_control_hybrid
source install/setup.bash
```

### Run the motor CAN node (I/O)
```bash
source /opt/ros/humble/setup.bash
source <your_ws>/install/setup.bash
ros2 run motor_control_hybrid python_can_node
```

### Run the control node (if used)
```bash
source /opt/ros/humble/setup.bash
source <your_ws>/install/setup.bash
ros2 run motor_control_hybrid cpp_control_node
```

---

## Examples: publishing commands

### Velocity (subset)
```bash
ros2 topic pub -r 50 /motor_commands motor_control_interfaces/msg/MotorCommand \
"{joint_name:['shoulder_pitch','wrist_roll'], mode:[0], velocity:[0.5,-0.2]}"
```

### Position (position=0 is valid)
```bash
ros2 topic pub -1 /motor_commands motor_control_interfaces/msg/MotorCommand \
"{joint_name:['shoulder_pitch'], mode:[1], position:[0.0], velocity:[0.5], acceleration:[1.0]}"
```

### Motion (kp/kd)
```bash
ros2 topic pub -r 200 /motor_commands motor_control_interfaces/msg/MotorCommand \
"{joint_name:['elbow_pitch'], mode:[2], position:[0.3], velocity:[0.0], torque:[0.0], kp:[40.0], kd:[1.5]}"
```

### Enable / Disable
```bash
ros2 topic pub -1 /motor_commands motor_control_interfaces/msg/MotorCommand \
"{joint_name:['shoulder_pitch','elbow_pitch'], mode:[3]}"
```

```bash
ros2 topic pub -1 /motor_commands motor_control_interfaces/msg/MotorCommand \
"{joint_name:['shoulder_pitch','elbow_pitch'], mode:[4]}"
```

---

## Files referenced by this README

- `README.md` — this document
- `system.struct.png` — rendered architecture diagram used in README
- `System Structurepng.drawio` — editable diagram source (draw.io / diagrams.net)
