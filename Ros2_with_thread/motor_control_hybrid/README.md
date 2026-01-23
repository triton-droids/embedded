# Motor Control Hybrid Architecture

Hybrid motor control system: Python CAN communication + C++ real-time control

## Architecture Overview

```
┌─────────────────────────────────────────┐
│         ROS2 Communication Layer (DDS)   │
└─────────────────────────────────────────┘
              ↓              ↓
    [Python CAN Node]  [C++ Control Node]
    /joint_states      /joint_commands
         ↓                    ↑
    CAN Bus ←─────────────────┘
```

### Components

1. **Python CAN Node** (`python_can_node.py`)
   - Independent CAN RX thread: Continuously receives motor status frames
   - Independent CAN TX thread: Reads commands from queue and sends
   - ROS2 interface: Publishes `/joint_states`, subscribes to `/joint_commands`
   - Uses `robstride_dynamics` SDK

2. **C++ Control Node** (`cpp_control_node.cpp`)
   - Subscribes to `/joint_states` (from Python CAN node)
   - Real-time control loop (50Hz)
   - Optional: RL policy inference (ONNX Runtime)
   - Publishes `/joint_commands` (to Python CAN node)

3. **Legacy Debug Node** (`motor_control_node_debug`)
   - Provides `/robstride_joint_control` service for configuration commands
   - Can run alongside the hybrid system for debugging and setup
   - Shares CAN bus with Python CAN node

## Dependencies

### Python Dependencies
- `rclpy`
- `sensor_msgs`
- `robstride_dynamics` (needs to be installed)

### C++ Dependencies
- `rclcpp`
- `sensor_msgs`
- Optional: `onnxruntime` (for RL inference)

## Building

```bash
cd /home/rcli/Github/embedded/embedded-humanoid-ros2/Ros2_with_thread
colcon build --packages-select motor_control_hybrid
source install/setup.bash
```

## Running

### Basic Launch

```bash
ros2 launch motor_control_hybrid hybrid_control.launch.py
```

### Launch with Parameters

```bash
ros2 launch motor_control_hybrid hybrid_control.launch.py \
    motor_config_file:=/path/to/motors.yaml \
    control_rate_hz:=50.0 \
    enable_rl:=false
```

### Parameters

- `motor_config_file`: Path to motor configuration YAML file
- `control_rate_hz`: Control loop frequency (default: 50.0 Hz)
- `enable_rl`: Enable RL policy (default: false)
- `rl_model_path`: Path to RL model file (ONNX format)

## Configuration File

Motor configuration file format (`config/motors.yaml`):

```yaml
motor_control_node:
  ros__parameters:
    default_can_interface: "can0"
    default_master_id: 255

    motors:
      joint_name:
        can_interface: "can0"
        master_id: 255
        motor_id: 1
        actuator_type: 2
```

## Data Flow

1. **State Update**:
   - CAN RX thread continuously receives motor status
   - Updates shared state buffer
   - ROS2 timer reads from buffer and publishes `/joint_states`

2. **Command Execution**:
   - C++ control node subscribes to `/joint_states`
   - Computes control output (or RL inference)
   - Publishes `/joint_commands`
   - Python CAN node receives commands, enqueues them
   - CAN TX thread reads from queue and sends to CAN bus

## Performance Characteristics

- **CAN RX Latency**: < 10ms (independent thread, non-blocking)
- **ROS2 Communication Latency**: 2-5ms (topic transmission)
- **Control Loop Jitter**: < 1ms (C++ real-time thread)
- **Total Latency**: ~5-15ms (suitable for 50Hz control)

## Extensibility

### Adding RL Policy

1. Export trained model to ONNX format
2. Integrate ONNX Runtime in C++ control node
3. Call inference in `control_loop()`

### Adding Other Sensors

- IMU: Publish to `/imu_data`, C++ node subscribes and fuses
- Vision: Publish to `/vision_features`, C++ node subscribes and fuses
- Voice: Publish to `/voice_commands`, C++ node subscribes and processes

## Troubleshooting

### Python CAN Node Cannot Import Motor Driver

Ensure `motor_control_node` package is built and in Python path:
```bash
colcon build --packages-select motor_control_node
source install/setup.bash
```

### CAN Communication Failure

Check if CAN interface is configured:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### Control Node Not Receiving States

Check if topics are publishing correctly:
```bash
ros2 topic echo /joint_states
ros2 topic echo /joint_commands
```

## Development Recommendations

1. **Real-time Optimization**:
   - Use independent threads for CAN RX/TX to avoid blocking
   - Use fixed-period timer in C++ control node
   - Consider using real-time scheduling (SCHED_FIFO)

2. **Safety**:
   - Add command limits
   - Add timeout detection
   - Add emergency stop mechanism

3. **Debugging**:
   - Use `ros2 topic echo` to view messages
   - Use `ros2 node info` to view node connections
   - Add logging output

## License

MIT
