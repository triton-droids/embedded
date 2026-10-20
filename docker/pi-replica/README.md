# AlohaMini Pi replica (DRAFT)

> **Draft, not yet verified against the real Pi.** This image is built from the
> documented LeRobot-on-Pi install pattern. Until `pi_inventory.sh` has been run on
> the AlohaMini2 Pi and its output reconciled with the build args below, treat this
> as an approximation, not a replica. Tracking issue: #41.

Dev environment approximating the AlohaMini2 Raspberry Pi software, so LeRobot and
AlohaMini code can be run and tested without the robot.

- Base: Ubuntu 24.04 (arm64; built and tested on Apple Silicon only)
- Python 3.12 in a conda env named `lerobot`
- ROS2 Jazzy (`ros-base`)
- AlohaMini fork of LeRobot ([liyitenga/lerobot_alohamini](https://github.com/liyitenga/lerobot_alohamini)), installed editable with `[feetech]` extras, plus `pyzmq`

## Build and run

```bash
docker build -t alohamini-pi .          # about 15 minutes
docker run -it --rm alohamini-pi        # shell, lerobot env and ROS sourced
docker run --rm alohamini-pi python -c "import lerobot; print(lerobot.__file__)"
```

Each `docker run --rm` starts a fresh container. Put permanent changes in the
Dockerfile and rebuild.

## Build args to reconcile with the Pi

| Arg | Current default | Confirm on the Pi with |
| --- | --- | --- |
| `LEROBOT_REPO` | AlohaMini fork | `git remote -v` in the Pi's lerobot checkout |
| `LEROBOT_REF` | `ec65318f` (fork `main`, 2026-10-09) | `git rev-parse HEAD` in the same checkout |
| `PYTHON_VERSION` | `3.12` (fork requires >= 3.12) | `conda run -n lerobot python --version` |
| `INSTALL_ROS` / `ROS_DISTRO` | `1` / `jazzy` | `ls /opt/ros` |

The base image (Ubuntu 24.04) must also match the Pi's `/etc/os-release`; Jazzy
requires 24.04 and Humble requires 22.04.

Override at build time, e.g. `docker build --build-arg INSTALL_ROS=0 -t alohamini-pi .`

## Verified vs unverified

| Item | Status |
| --- | --- |
| Clean build, `import lerobot` (from `/root/lerobot/src/lerobot/`) | Verified on arm64 |
| ROS2 Jazzy, `zmq`, `scservo_sdk` import | Verified on arm64 |
| `robots/alohamini/` present in the fork | Verified |
| Same LeRobot source/commit, Python, OS and ROS as the Pi | **Unverified** |
| Boot services, udev rules (`/dev/am_*` symlinks) | **Unverified / missing** |
| Servo and camera access | Not testable on macOS (Docker Desktop does not pass USB/serial); on Linux use `--device` |
| x86_64 build | Not run |

## Inventorying the real Pi

`pi_inventory.sh` is read-only. It records OS, hardware, Python/conda, LeRobot
checkouts (remote, commit, local changes), ROS2, enabled services and manually
installed packages. It does not dump environment variables, SSH keys or Tailscale
state, and it drops lines that look like tokens or passwords from `.bashrc` and
crontab output.

```bash
scp pi_inventory.sh <pi>:~ && ssh <pi> 'bash pi_inventory.sh'
scp <pi>:~/pi_inventory.txt .
```

Read `pi_inventory.txt` before sharing it. It is gitignored here; do not commit it.
