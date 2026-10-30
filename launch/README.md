# Simulation launch entry points

Run from the repository root after building and sourcing `install/setup.bash`.

| Task | Command |
|---|---|
| MuJoCo simulation | `ros2 launch humanoid_sim sim.launch.py` |
| Optional vision within simulation | `ros2 launch humanoid_sim sim.launch.py enable_vision:=true` |
| Leg display from measured feedback | `ros2 launch launch/display.launch.py use_joint_state_gui:=false` |
| Detector/tracker using existing simulated camera topics | `ros2 launch launch/head_tracking.launch.py start_camera:=false start_agent:=false` |
| Tracker-only launch | `ros2 launch launch/head_tracking.launch.py start_camera:=false start_agent:=false start_detector:=false` |

The default simulation retains ONNX, C++ forwarding, Web monitoring and safety.
It starts on a virtual gantry with motors disabled. See the
[simulation guide](../simulation/humanoid_sim/README.md) for autostart, reset,
viewer controls, isolated DDS defaults and current policy limitations.

Launch source and defaults are preserved from the original workspace. The
shared vision launch still defaults to physical camera/Agent startup when used
directly: disable both explicitly as above. The MuJoCo launch already supplies
those overrides and provides camera topics and the head-servo backend.
Vision requires a full `humanoid_vision` build for detection; `TRACK_ONLY=ON`
permits the tracker without the detector SDKs. Full detection still requires
OpenCV 4.5.x as documented by the package.

The physical `system.launch.py`, real CAN control launches and camera/Agent
startup recipe are retained in [hardware/](../hardware/README.md), outside the
default build. They are historical recipes, not installed simulation commands.
