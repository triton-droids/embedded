# Humanoid ROS 2 Simulation

MuJoCo simulation of a ten-joint humanoid leg robot with ONNX policy inference,
C++ command scheduling, safety supervision, Web monitoring and optional visual
head tracking. Physical-device code is archived in [hardware/](hardware/README.md).
MPC/WBC is not implemented.

## Quick start

From the repository root on ROS 2 Jazzy, or the supplied Humble dev container:

```bash
source scripts/setup_rosenv.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --executor sequential --packages-up-to humanoid_sim
source install/setup.bash
ros2 launch humanoid_sim sim.launch.py
```

The default simulation starts held on a virtual gantry with motors disabled.
Add `auto_start:=true` to enable and release after fresh control is available.
The connected policy always passes through `cpp_control_node`; Web monitoring
and safety remain in the full stack.

Read the [simulation guide](simulation/humanoid_sim/README.md) for viewer controls,
headless runs, reset, launch arguments and current limitations. In particular,
the current policy is sensitive to feedback/command delay and does not reliably
complete its reference walk. The dynamics model remains a candidate model.

For an existing checkout, use fresh build/install directories after this source
reorganization. Old install trees can retain removed hardware executables and
symlinks to previous source locations.

## Workspace layout

| Path | Responsibility |
|---|---|
| [`simulation/humanoid_sim/`](simulation/humanoid_sim) | MuJoCo world, simulated motors, IMUs, head servo, camera and dynamics tests. |
| [`humanoid_control/motor_control_interfaces/`](humanoid_control/motor_control_interfaces) | Shared command, feedback and operation messages/services. |
| [`humanoid_control/motor_control_hybrid/`](humanoid_control/motor_control_hybrid) | ONNX policy, C++ scheduler, fake motor, visualization relay and simulation registry/calibration helpers. |
| [`humanoid_control/humanoid_safety/`](humanoid_control/humanoid_safety/README.md) | Safety supervisor, Web telemetry and advisory collision checking. |
| [`libraries/control_core/`](libraries/control_core/README.md) | ROS-independent C++ command core and operation state machines; optional Python bindings. |
| [`humanoid_leg_description/`](humanoid_leg_description/README.md) | Leg URDFs and meshes used by simulation, display and collision checking. |
| [`humanoid_vision/`](humanoid_vision) | Optional detection, depth projection and head tracking on simulated camera topics. |
| [`launch/`](launch/README.md) | Existing leg display and head-tracking launch entry points. |
| [`scripts/`](scripts) | Simulation environment, build and model-conversion helpers. |
| [`docs/`](docs/README.md) | Simulation guides, shared control contracts and architecture notes. |
| [`hardware/`](hardware/README.md) | Archived physical CAN backends, firmware, tuning tools, arm model, device launches and deployment references. |
| [`legacy/`](legacy/README.md) | Archived SDK/Torch/WebSocket adapters, MoveIt demos and superseded implementations. |

## Default build boundaries

- `hardware/` and `legacy/` are excluded with `COLCON_IGNORE`.
- No physical CAN node, serial IMU reader, micro-ROS Agent or RealSense driver
  is installed or launched by the default simulation composition.
- The simulation build does not fetch ENCOS drivers or require RobStride,
  serial, gRPC, WebSocket or Torch Python dependencies.
- `motors.yaml`, `MotorSettings` and ankle conversion stay in shared control
  code because the simulated backend uses their registry and feedback semantics.
- Existing launch source and defaults are preserved. The simulation explicitly
  passes `start_camera:=false start_agent:=false` to the shared vision launch.
  When invoking that launch directly on simulated topics, pass both flags too;
  its original standalone defaults still request physical camera/Agent startup.
- Build output, logs, PlatformIO output and local compilation databases are
  ignored. Use `/tmp` or ignored `local_test/` for disposable checks.
- Model exports, mesh variants and protocol source archives are retained.

## Guides

- [MuJoCo simulation](simulation/humanoid_sim/README.md)
- [Launch shortcuts](launch/README.md)
- [Documentation index](docs/README.md)
- [Control watchdogs and command lifetime](docs/control_watchdogs.md)
- [ONNX policy contract](docs/tracking_policy_ros2.md)
- [Development container](docs/devcontainer.md)
- [Archived physical robot reference](hardware/README.md)
