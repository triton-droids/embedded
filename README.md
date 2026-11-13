# Humanoid ROS 2 Simulation

MuJoCo simulation of a ten-joint humanoid leg robot running the robot's own
software stack: an ONNX tracking policy or a model-based walking controller
(LIPM MPC + TSID), the C++ command scheduler, safety supervision and Web
monitoring, and a camera ball chase (YOLO on TensorRT or Isaac ROS) with
optional localization from the body IMU and stereo visual odometry.

## Quick start

ROS 2 Jazzy, or Humble (Jetson / dev container), from the repository root:

```bash
source scripts/setup_rosenv.sh        # first time: Python env, Pinocchio, patched TSID
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential \
  --packages-up-to humanoid_sim humanoid_ball_chase walking_controller humanoid_localization \
  --cmake-args -DTRACK_ONLY=ON      # humanoid_vision: tracker only, no OpenCV detector build
source install/setup.bash
```

Run:

```bash
ros2 launch humanoid_sim sim.launch.py             # policy simulation, viewer, robot held on a gantry
ros2 launch humanoid_sim sim.launch.py auto_start:=true   # enable motors and release when control is ready
scripts/ball_chase_walk.sh scene:=hard             # walking robot chases the football (headless)
```

Jetson extras (see the guides below): `scripts/build_ball_chase_engines.sh` for
the GPU detector, `scripts/run_cuvslam.sh` + `enable_vio:=true` for localization,
`scripts/run_isaac_yolo.sh` + `vision_detector:=isaac` for the Isaac ROS detector.

## Guides

| Topic | README |
|---|---|
| Simulator: launch arguments, viewer, controls, stereo, limitations | [simulation/humanoid_sim](simulation/humanoid_sim/README.md) |
| Ball chase: camera, detectors (TensorRT, Isaac ROS), A*, VIO switch | [simulation/humanoid_ball_chase](simulation/humanoid_ball_chase/README.md) |
| Model-based walking controller | [humanoid_control/walking_controller](humanoid_control/walking_controller/README.md) |
| Localization: body IMU + cuVSLAM stereo VO | [humanoid_control/humanoid_localization](humanoid_control/humanoid_localization/README.md) |
| WBC simulation path (policy + TSID) | [humanoid_control/wbc_control_node](humanoid_control/wbc_control_node/README.md) |
| Safety, Web monitoring | [humanoid_control/humanoid_safety](humanoid_control/humanoid_safety/README.md) |
| C++ command core | [libraries/control_core](libraries/control_core/README.md) |
| Leg URDF and meshes | [humanoid_leg_description](humanoid_leg_description/README.md) |
| Evaluations | [VIO and Isaac trials](evaluation/vio_isaac_trials/README.md), [fall predictor](evaluation/fall_predictor/README.md) |
| Documentation index, architecture, build boundaries | [docs](docs/README.md) |
| Physical robot (archived) and legacy code | [hardware](hardware/README.md), [legacy](legacy/README.md) |
