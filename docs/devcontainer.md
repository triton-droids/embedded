# Dev container (ROS 2 Humble without a native install)

`.devcontainer/` provides an Ubuntu 22.04 / ROS 2 Humble environment matching
the Jetson, for laptops that are not on Ubuntu or should not have ROS installed.
It targets **Linux hosts** with Docker; host networking and X11 forwarding do
not behave the same way under Docker Desktop on macOS/Windows.

## Start

VS Code: install the Dev Containers extension, open the repository, and run
**Dev Containers: Reopen in Container**.

CLI:

```bash
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash
```

The first build downloads several GB. `post-create.sh` runs rosdep, creates
`rosenv/` (Python 3.10) and makes every new shell source ROS, `rosenv` and
`install/setup.bash`. Then build:

```bash
colcon build --symlink-install
source install/setup.bash
```

`rosenv/`, `build/` and `install/` live in the repository folder but are only
valid inside the container. Delete them before building on another system.

## Differences from the Jetson setup

- **Torch is not installed** (only the legacy `policy_bridge_node` uses it).
  To add a CPU-only build, run `INSTALL_TORCH=1 bash .devcontainer/post-create.sh`
  inside the container.
  `rosenv/.requirements.stamp` is written so `scripts/setup_rosenv.sh` does not
  pull the multi-GB CUDA wheel.
- **Vision is CPU-only**: run `ball_detection_node` with `use_cuda:=false`.
  TensorRT engines are Jetson-specific. The micro-ROS Agent is not installed.
- **Isolated ROS network by default**: `ROS_DOMAIN_ID=42` and
  `ROS_LOCALHOST_ONLY=1`, so fake-motor tests cannot reach the robot (domain 0).
  To view robot topics from RViz in the container, change both values in
  `devcontainer.json` and rebuild; see [remote RViz](remote_rviz.md).

## GUI (RViz, MoveIt)

The container uses the host X server through `/tmp/.X11-unix` and `DISPLAY`
(XWayland on Wayland desktops). Before each start, `init-host.sh` copies the
host's X auth cookie to `~/.cache/triton-devcontainer/xauth`; that directory
is mounted so the container's `XAUTHORITY` always sees the current cookie. After logging out and back in, restart the
container to refresh it. If RViz still reports `could not connect to display`,
allow local clients on the host and retry:

```bash
xhost +SI:localuser:$(id -un)
```

RViz uses software rendering by default; `MESA: error: Failed to query drm
device` / `libGL error` lines at startup are expected and harmless. For GPU
rendering, uncomment `--device=/dev/dri` in `devcontainer.json`.

## Hardware

Serial devices are not passed through by default. Uncomment
`--device=/dev/ttyACM0` (only if the device exists, or the container will not
start). CAN interfaces are host network devices and are visible through host
networking, but real motor commissioning belongs on the robot; see
[activate the tracking system](activate_tracking_system.md#8-real-motors--configuration-and-feedback-commissioning).

## First test

The no-hardware fake-motor/RViz check in
[remote RViz: fake motor verification](remote_rviz.md#fake-motor-verification)
works unchanged inside the container.
