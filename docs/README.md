# Simulation documentation

## Latest checkpoint

- [2026-11-13 goal-shot tests, successful video and publication scope](reports/2026-11-13-goal-shot.md)
- [Goal-shot / learned-motion continuation handoff](handoff-goal-shot-2026-11-13.md)

These records describe the tested Jetson working tree. Associated recent code is
not included in this documentation-only commit.

## Run and inspect

- [Workspace overview](../README.md)
- [MuJoCo setup, launch options, controls and limitations](../simulation/humanoid_sim/README.md)
- [Launch shortcuts](../launch/README.md)
- [Development container](devcontainer.md)
- [Remote RViz and fake-motor visualization](remote_rviz.md)
- [Safety, Web monitoring and ROS visualization](../humanoid_control/humanoid_safety/README.md)

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
