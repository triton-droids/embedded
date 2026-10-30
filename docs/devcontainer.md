# Simulation dev container

`.devcontainer/` provides Ubuntu 22.04 / ROS 2 Humble for Linux hosts with Docker,
host networking and X11. It installs shared ROS/vision dependencies and the
ONNX/MuJoCo Python environment. Physical CAN drivers, MoveIt demos, RealSense
startup and MCU tools are archived outside the default build.

## Start

In VS Code, use **Dev Containers: Reopen in Container**, or run:

```bash
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash
```

The first build downloads the base image and dependencies. `post-create.sh`
runs rosdep on active packages, creates `rosenv/`, and configures new shells.
Then build and run:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --executor sequential --packages-up-to humanoid_sim
source install/setup.bash
ros2 launch humanoid_sim sim.launch.py
```

The source-mounted `rosenv/`, `build/` and `install/` contain host-specific
Python and compiled paths. Use fresh directories when switching between host
and container builds, or after the archive reorganization.

## Rendering and ROS

The container forwards `/tmp/.X11-unix` and `DISPLAY`. `init-host.sh` copies the
host X auth cookie into `~/.cache/triton-devcontainer/xauth`; restart the
container after logging out/in to refresh it. Software rendering is the default;
optional `/dev/dri` passthrough enables host GPU rendering where supported.

DDS defaults to `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`. Vision defaults to
CPU operation; TensorRT/CUDA dependencies remain optional and host-specific.
The simulation supplies IMU and camera data, so no serial device or micro-ROS
Agent is needed. The image does not build or fetch the ENCOS driver.

See [the simulation guide](../simulation/humanoid_sim/README.md) for headless
rendering and optional camera operation. The original hardware-oriented image
recipe is preserved under [hardware/devcontainer](../hardware/devcontainer).
