# ROS2-Free Robot SDK Prototype

This prototype keeps ROS2 behind an internal adapter and exposes a normal robot SDK surface.

Recommended transport split:

- **gRPC + Protobuf** for commands and low-rate queries.
- **ZeroMQ PUB/SUB** for local robot state monitoring.
- **WebSocket later** for browser UI dashboards.

```text
Python/C++ SDK client
        |
        | commands: gRPC + Protobuf
        | state:    ZeroMQ PUB/SUB JSON
        v
SDK gateway process
        |
        | internal adapter, not exposed to SDK users
        v
Existing ROS2 HTTP gateway or ROS2 node bridge
        |
        v
ROS2 sensors, inference/RL, safety, control, CAN I/O
```

SDK users should see methods like `enable_robot()`, `set_mode()`, and `stream_robot_state()`, not ROS2 topics, services, actions, QoS, package names, or launch files.

## Folder Structure

```text
sdk_prototype/
  README.md
  requirements.txt
  proto/
    robot_sdk.proto
  python/
    generate_grpc_python.sh
    robot_sdk_demo/
      model.py
      grpc_server.py
      grpc_client.py
      zmq_state.py
      hybrid_gateway.py
      hybrid_client.py
      robot_sdk_pb2.py       # generated
      robot_sdk_pb2_grpc.py  # generated
```

## Run

Install dependencies:

```bash
python3 -m pip install -r sdk_prototype/requirements.txt
```

Regenerate gRPC code after editing the proto:

```bash
bash sdk_prototype/python/generate_grpc_python.sh
```

Terminal 1:

```bash
python3 -m sdk_prototype.python.robot_sdk_demo.hybrid_gateway
```

Terminal 2:

```bash
python3 -m sdk_prototype.python.robot_sdk_demo.hybrid_client
```

The gateway exposes:

- `127.0.0.1:50051` for gRPC command/query calls.
- `tcp://127.0.0.1:5556` for ZeroMQ state stream.

## ROS2 Motor Gateway

The ROS2-side motor gateway is:

```bash
ros2 run motor_control_hybrid motor_sdk_gateway_node
```

It exposes `127.0.0.1:50052` and bridges SDK motor RPCs to ROS2:

- `EnableMotors` -> publishes `motor_control_interfaces/MotorCommand` with `MODE_ENABLE`.
- `DisableMotors` -> publishes `MODE_DISABLE`.
- `SetMotorVelocity` -> publishes `MODE_VELOCITY`.
- `SetMotorPosition` -> publishes `MODE_POSITION`.
- `SetMotorMit` -> publishes `MODE_MOTION`.
- `GetMotorStatus` -> reads latest `/joint_states` and `/motor_status`.

Client example:

```python
from sdk_prototype.python.robot_sdk_demo.grpc_client import MotorGrpcClient

motors = MotorGrpcClient("127.0.0.1:50052")

joints = ["test_joint", "test_joint2"]
print(motors.enable_motors(joints))
print(motors.set_motor_velocity(joints, [0.5, -0.5], [10.0, 10.0]))
print(motors.set_motor_position(joints, [0.2, -0.2], [1.0, 1.0], [40.0, 40.0], [1.5, 1.5]))
print(motors.set_motor_mit(joints, [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [40.0, 40.0], [1.5, 1.5]))
print(motors.get_motor_status(joints))
print(motors.disable_motors(joints))
```

## Test Without Motors

Use the fake ROS2 motor node instead of the CAN node:

```bash
source /opt/ros/humble/setup.bash
cd Ros2_with_thread
colcon build --packages-select motor_control_interfaces motor_control_hybrid
source install/setup.bash
```

Terminal 1:

```bash
ros2 run motor_control_hybrid fake_motor_node
```

Terminal 2:

```bash
ros2 run motor_control_hybrid motor_sdk_gateway_node
```

Terminal 3:

```bash
python3 -m sdk_prototype.python.robot_sdk_demo.motor_client_example
```

Optional browser visualization:

```bash
ros2 run motor_control_hybrid double_pendulum_websocket_node
```

Open `http://127.0.0.1:8765` to see `test_joint` and `test_joint2` as a double pendulum. The page receives `/joint_states` over WebSocket and can send enable, disable, and position commands back to ROS2 through `/motor_commands`.

This exercises the full SDK command path:

```text
MotorGrpcClient
  -> gRPC MotorControl
  -> motor_sdk_gateway_node
  -> /motor_commands
  -> fake_motor_node
  -> /joint_states + /motor_status
  -> GetMotorStatus
```

## API Shape

```python
robot.enable_robot()
robot.set_mode("velocity")
robot.load_policy("walk_v1", "/opt/policies/walk_v1.onnx")
robot.start_policy("walk_v1")
robot.set_velocity_command(vx_mps=0.2, vy_mps=0.0, wz_radps=0.1)
status = robot.get_robot_status()

for state in robot.stream_robot_state():
    print(state)

robot.stop_policy()
robot.disable_robot()
```

## Production Direction

Keep one internal robot state model and publish it to multiple adapters:

| Flow | Transport | Why |
| --- | --- | --- |
| Commands | gRPC unary RPC | Typed contract, generated Python/C++ clients, deadlines, explicit errors. |
| Status query | gRPC unary RPC | Low-rate typed request/response. |
| State stream | ZeroMQ PUB/SUB | Low-overhead local fanout for SDK clients, logs, and debugging tools. |
| Browser UI | WebSocket bridge | Native browser support and easier auth/session integration. |

The WebSocket server should be an adapter, not the source of truth. It should subscribe to the same state model and forward UI commands through the gRPC command client.

## Recommendation

Use this hybrid layout as the SDK direction:

- Public command API: **gRPC + Protobuf**.
- Local state stream: **ZeroMQ PUB/SUB**, JSON first, Protobuf payload later if schema drift becomes a problem.
- Browser UI: **WebSocket adapter** layered on top of the gateway.
