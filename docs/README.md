# Simulation documentation

## Run and inspect

- [Workspace overview](../README.md)
- [MuJoCo setup, launch options, controls and limitations](../simulation/humanoid_sim/README.md)
- [Launch shortcuts](../launch/README.md)
- [Development container](devcontainer.md)
- [Remote RViz and fake-motor visualization](remote_rviz.md)
- [Safety, Web monitoring and ROS visualization](../humanoid_control/humanoid_safety/README.md)
- [Ball chase: camera, football detection, Isaac ROS detector, A*](../simulation/humanoid_ball_chase/README.md)
- [Model-based walking controller package](../humanoid_control/walking_controller/README.md)
- [Localization: body IMU + cuVSLAM stereo VO](../humanoid_control/humanoid_localization/README.md)
- [VIO and Isaac ROS detection trials](../evaluation/vio_isaac_trials/README.md)

## Shared control and models

- [Architecture notes](current_architecture.md)
- [Command deadlines and watchdogs](control_watchdogs.md)
- [ONNX policy integration and model contract](tracking_policy_ros2.md)
- [Policy performance notes](tracking_policy_performance.md)
- [Motor registry and model alignment audit](motor_configuration_audit.md)
- [Self-collision monitor](self_collision_monitor.md)
- [Active C++ control library](../libraries/control_core/README.md)
- [Leg model and mesh assets](../humanoid_leg_description/README.md)
- [Vision build and algorithm reference](humanoid_vision_instructions.md)
- [Vision-control roadmap](vision_control_plan.md) (planned work)
- [Model-based walking controller](model_based_walking_design.md) (LIPM MPC + TSID, walks in simulation; see the package README)

## Archives

- [Physical robot sources, firmware and deployment references](../hardware/README.md)
- [Original complete interface/deployment reference](../hardware/docs/ros_interfaces.md)
- [Historical software and relocation notes](../legacy/README.md)
- [Historical architecture image](architecture/historical_system.png)
- [2026-10-19 update](updates/2026-10-19.md)

Some retained architecture/performance guides describe the original mixed
hardware/simulation workspace. Hardware launch options in those historical
sections are not installed in this simulation-only checkout. Start with the
simulation guide for supported execution paths.

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
