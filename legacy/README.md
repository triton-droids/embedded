# Historical software archive

These retained sources are outside the default simulation build and install.
Physical robot sources are grouped separately in [hardware/](../hardware/README.md).

| Directory | Purpose |
|---|---|
| [sdk_prototype](sdk_prototype/README.md) | Old SDK, gRPC client, browser tools and examples. |
| [motor_adapters](motor_adapters/README.md) | Former ROS SDK gateway, WebSocket demo and Torch policy bridge, with protocol bindings, config and tests. |
| [control_core_velocity](control_core_velocity/README.md) | Superseded velocity-only C++ controller. |
| [http_gateway_examples](http_gateway_examples/README.md) | Former HTTP clients; not examples of the current control library. |
| [motor_control](motor_control/README.md) | Historical threaded CAN backup. |
| [humanoid_leg_moveit_config](humanoid_leg_moveit_config) | Historical leg MoveIt demo. |
| [humanoid_arm_moveit_config](humanoid_arm_moveit_config) | Historical arm MoveIt demo. |

`COLCON_IGNORE` excludes this archive from normal workspace discovery.
Original ROS package/module names in archived examples describe their former
packaging; they are not installed executables in the simulation workspace.

The active ROS-independent library moved from `Interfaces/control_core/` to
[libraries/control_core/](../libraries/control_core/README.md). Its C++ behavior
is unchanged. Current ROS public headers come from that library; the archived
velocity-only headers are no longer installed alongside conflicting definitions.

The former root `system.struct.png` is retained as
[a historical architecture image](../docs/architecture/historical_system.png).
Build output and logs are disposable; model snapshots and original source assets
are retained. Rebuild in fresh build/install directories after these relocations
so stale installed hardware executables and old absolute source paths do not
survive from a previous workspace build.
