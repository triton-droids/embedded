# AlohaMini Pi replica

Docker image matching the software on the AlohaMini2 Raspberry Pi, so code can be tried against the
same OS, user and Python without the robot. Tracking issue: #41.

Matched to `pi_inventory.txt` taken on 2026-11-09 from the Pi (hostname `tdroids-desktop`).

## What the Pi has (2026-11-09)

| Item | On the Pi | In the image |
| --- | --- | --- |
| OS | Ubuntu 24.04.4 LTS, arm64 | `ubuntu:noble-20260905`, the newest 24.04.4 base image |
| Hardware | Raspberry Pi 4 Model B Rev 1.5, 3.7 GiB RAM, 58 GB SD card | n/a |
| Kernel | `6.8.0-1065-raspi` | Docker host's kernel (cannot match) |
| User | `tdroids`, uid 1000, groups adm dialout cdrom sudo dip plugdev users lpadmin sambashare | Same. Passwordless sudo in the image only |
| Python | Miniforge at `~/miniforge3`, base Python 3.13.13 | Same path and version |
| Conda envs | `base`, `so101` | `base`. `so101` only with `--build-arg SO101_PYTHON=...` (contents unknown, see below) |
| LeRobot | **No checkout found, no `lerobot` env** | Not installed (opt-in, see below) |
| ROS2 | **Not installed** (`/opt/ros` missing, no `ros-` packages) | Not installed (opt-in) |
| Docker | **Not installed** | n/a |
| v4l-utils | **Not installed** | Not installed (`DEV_TOOLS=1` adds it) |
| `.bashrc` | conda + mamba init, `ALOHA_WEB_HOST=100.113.252.70`, `ALOHA_WEB_PORT=8000` | Same |
| Desktop | Ubuntu desktop (GNOME, gdm), snaps | Left out |
| Services | ssh, tailscaled, NetworkManager, cups, avahi, ... | Left out (containers do not run systemd) |
| USB | VIA hub and a Kingston USB stick only. **The Pi was not connected to the robot** when inventoried, so camera and servo board are unknown | n/a |
| udev | Only snap and ModemManager rules. **No `/dev/am_*` rules** (these are files on the SD card, so this holds even when the robot is connected) | n/a |
| Tailscale | `tdroids-desktop`, 100.113.252.70, tailnet account `jetsonTdroids@` | n/a |

## Build and run

```bash
docker build -t alohamini-pi .            # a few minutes
docker run -it --rm alohamini-pi          # login shell as tdroids
```

Build args:

| Arg | Default | Use |
| --- | --- | --- |
| `SO101_PYTHON` | empty | Python version of the Pi's `so101` env, once known. Creates an empty env with that Python |
| `INSTALL_LEROBOT` | `0` | `1` adds the AlohaMini LeRobot fork (`LEROBOT_REPO`, `LEROBOT_REF`) in a separate `lerobot` env with Python `LEROBOT_PYTHON` (3.12). **Not on the Pi**: this is the planned setup, not a replica |
| `INSTALL_ROS` / `ROS_DISTRO` | `0` / `jazzy` | ROS2 ros-base. **Not on the Pi** |
| `DEV_TOOLS` | `0` | build-essential, cmake, v4l-utils. **Not on the Pi** |

## Still unknown

The 2026-11-09 inventory did not capture these. `pi_inventory.sh` has been updated to collect them on the next run.

- **What is in `so101`**: Python version and packages. The old script only looked for a `lerobot` env. It now
  exports `python --version` and `pip freeze` for every env.
- **Where the robot code lives**: the old script only searched `~/lerobot*` and `~/AlohaMini*`. It now searches
  `$HOME` 4 levels deep for any git checkout with lerobot or aloha in its path, and lists `$HOME`.
- **What serves `ALOHA_WEB_PORT=8000`**: the script now records listening ports (`ss -ltnp`).
- **Camera and servo board**: the Pi was not connected to the robot. Re-run the inventory with it connected. `v4l2-ctl` is missing, so the script now also reads camera names from
  `/sys/class/video4linux`. `/dev/video10`-`31` are the Pi 4's built-in codec/ISP devices, not cameras.

## Inventorying the Pi

`pi_inventory.sh` is read-only. It does not dump environment variables, SSH keys or Tailscale state, and it
drops lines that look like tokens or passwords from `.bashrc` and crontab output.

```bash
scp pi_inventory.sh tdroids@100.113.252.70:~ && ssh tdroids@100.113.252.70 'bash pi_inventory.sh'
scp tdroids@100.113.252.70:~/pi_inventory.txt .
```

This needs your machine on the `jetsonTdroids@` tailnet, or use the USB stick at the console as before.
`pip freeze` output can include private package URLs, so read `pi_inventory.txt` before sharing it.
It is gitignored here; do not commit it.

## Verified

| Check | Result |
| --- | --- |
| Default image builds (arm64, Apple Silicon) | Yes |
| `/etc/os-release` | Ubuntu 24.04.4 LTS |
| `id` | uid=1000(tdroids) gid=1000(tdroids), same groups as the Pi |
| `python --version` (base) | Python 3.13.13, standard (not free-threaded) build |
| No ROS, no Docker, no v4l2-ctl, `lerobot` env absent | Yes, as on the Pi |
| `pi_inventory.sh` runs to completion inside the image | Yes |
| `INSTALL_LEROBOT=1` variant builds; `lerobot`, `zmq`, `scservo_sdk` import; `robots/alohamini/` present | Yes |
