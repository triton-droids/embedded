# stella_vslam (ROS2 Jazzy, arm64, headless)

stella_vslam for the AlohaMini2: one image that runs offline tests on the Mac and runs live on the Pi 4,
plus a USB-stick bundle that installs it on the Pi from the console, without SSH.

- Base: `ros:jazzy-ros-base` (Ubuntu 24.04), built for `linux/arm64`, the Pi 4's architecture and Apple Silicon's
- stella_vslam `0.7.0`, stella_vslam_ros `ros2` branch, socket_publisher `0.0.2`, all pinned in the `Dockerfile`
- Upstream ships a ROS2 **Humble** Dockerfile (`Dockerfile.socket`). This is a Jazzy port of it, with protobuf from apt
  instead of 3.6.1 built from source, and without backward-cpp. Two patches to stella_vslam_ros are applied at build
  time (duplicate ament env hook; `cv_bridge.h` -> `cv_bridge.hpp`), see the `Dockerfile`.

## Verified so far

### Mac, Apple Silicon (2026-11-03 and 2026-11-14)

| Check | Result |
|---|---|
| Both images build for linux/arm64 | Yes. `stella-vslam:arm64` is 2.65 GB |
| `run_tum_test.sh` (TUM freiburg1_xyz, offline) | 791-797 / 798 frames tracked, 19-22 keyframes, median 7.9-8.2 ms per frame |
| Accuracy on the same sequence (keyframe APE RMSE, `evo_ape -as`, 2026-11-14) | stella_vslam 10.4-10.8 mm (3 runs); ORB-SLAM3 8.6 mm (1 run). See [Accuracy](#accuracy-check-tum-freiburg1_xyz) |
| Robot stack (`robot/compose.yaml`) with a TUM image publisher in place of the camera | Viewer connects; SLAM initializes; `docker compose stop` saves the map, trajectories and track times |
| Localization mode against that saved map | Map loads; poses on `/run_slam/camera_pose` at about 23 Hz |

### AlohaMini2 Pi 4 (`tdroids-desktop`, 4 GB, Ubuntu 24.04.4; 2026-11-17)

| Check | Result |
|---|---|
| `make_usb_bundle.sh ... --with-docker-debs` to a folder, copied to the Pi with `scp` | Works |
| `install.sh` | Docker installed, both images loaded, systemd unit installed |
| TUM freiburg1_xyz, offline ([Pi benchmark](#pi-4-benchmark-offline)) | 793 / 798 frames tracked, 21 keyframes, **keyframe APE RMSE 9.9 mm** (same as the Mac) |
| Tracking time per frame, TUM, 30 fps input | **median 43.9 ms**, p90 51.5, p99 60.5. 96% of frames are over the 33 ms budget for 30 fps |
| Same with `--frame-skip 2` (simulates 15 fps) | 398 / 399 frames tracked, median 43.4 ms, inside the 66 ms budget for 15 fps |
| Thermal or power throttling during the runs | None: `vcgencmd get_throttled` = `0x0` before and after, 51 °C peak |
| Camera detected | `32e4:9320 HD USB Camera` (UVC, the ELP) at `/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_2020032001-video-index0` |
| Live camera -> `v4l2_camera` -> SLAM (systemd service, YUYV 640x480 30 fps) | Frames reach SLAM, median 41.7 ms each, so it keeps up with about 22-24 of the 30 fps. Clean `systemctl stop` saves the map and track times |

The Pi 4 is about 5.5x slower than the Mac. At default settings it can't keep up with 30 fps, but it fits 15 fps.
Accuracy doesn't depend on the machine.

**Not verified:** live **tracking**. In the live run the camera didn't move, so SLAM never initialized, and the config
still has placeholder intrinsics. Also not verified: the fisheye config with real calibration, the browser viewer on
the Pi, booting with `--enable`, and installing from a USB stick (the bundle was copied over the network). Accuracy
has only been measured on TUM, a handheld Kinect over a textured desk, not on the ELP fisheye or a moving base.
- Browser viewer (`Dockerfile.viewer`, stella-cv/socket_viewer on node 18): open `http://<host>:3001`
- License: stella_vslam is BSD-2-Clause; g2o is linked dynamically (ros-jazzy-libg2o) because of its LGPL part.

## Files

| File | What it is |
|---|---|
| `Dockerfile`, `entrypoint.sh` | SLAM image: core library, examples, ROS2 node, v4l2_camera, FBoW vocabulary |
| `Dockerfile.viewer` | Browser viewer server |
| `run_tum_test.sh` | Offline check on TUM freiburg1_xyz (Mac) |
| `calibrate_fisheye.py` | ELP fisheye calibration, run natively on the Mac |
| `config/elp_fisheye.yaml` | Camera config. **Placeholder intrinsics** until calibrated |
| `make_usb_bundle.sh` | Builds the images and writes the Pi install bundle to a folder or USB stick |
| `robot/` | What gets installed on the Pi: `install.sh`, `compose.yaml`, `slam.env`, `run_slam.sh`, systemd unit |

## 1. Build and test on the Mac

```bash
docker build --platform linux/arm64 --build-arg JOBS=6 -t stella-vslam:arm64 .
docker build --platform linux/arm64 -f Dockerfile.viewer -t stella-viewer:arm64 .
./run_tum_test.sh
```

`JOBS=6` needs about 8 GB of Docker memory. Use the default (2) on smaller VMs.

### Accuracy check (TUM freiburg1_xyz)

`run_tum_test.sh` scores the keyframe trajectory against TUM's motion-capture ground truth when
[evo](https://github.com/MichaelGrupp/evo) is installed:

```bash
python3 -m venv ~/evo-venv && ~/evo-venv/bin/pip install evo
EVO_APE=~/evo-venv/bin/evo_ape ./run_tum_test.sh
```

It uses `run_tum_rgbd_slam`, which takes timestamps from the dataset's `rgb.txt`. `run_image_slam` stamps frames with
wall-clock time, and evo then can't match the poses to `groundtruth.txt`.

To compare with ORB-SLAM3 on the same data (image from `../orbslam3`):

```bash
docker run --rm -v "$(pwd)/data:/workspace/data" orbslam3 bash -c '
mkdir -p /workspace/data/out_orb && cd /workspace/data/out_orb &&
/opt/ORB_SLAM3/Examples/Monocular/mono_tum /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /opt/ORB_SLAM3/Examples/Monocular/TUM1.yaml /workspace/data/rgbd_dataset_freiburg1_xyz'
cd data
~/evo-venv/bin/evo_ape tum rgbd_dataset_freiburg1_xyz/groundtruth.txt out_orb/KeyFrameTrajectory.txt -as
```

Results (camera path about 9 m):

| System | Where | Keyframes | APE RMSE | APE max |
|---|---|---|---|---|
| stella_vslam | Mac, 2026-11-14 | 20-22 | 10.4-10.8 mm (3 runs) | 18-24 mm |
| ORB-SLAM3 | Mac, 2026-11-14 | 51 | 8.6 mm (1 run) | 25 mm |
| stella_vslam | Pi 4, 2026-11-17 | 21 | 9.9 mm (1 run) | 23 mm |

The 2 mm difference is within run-to-run variation and comes from different keyframe sets, so the two are equivalent
here. `-as` applies a Sim(3) alignment (rotation, translation and scale) before measuring, because monocular SLAM has no
metric scale. These numbers measure trajectory shape, not scale. Keyframe counts are not a quality measure: the two
systems choose keyframes differently. Per-frame poses (`frame_trajectory.txt`) score worse (about 34 mm RMSE) because
they are not refined by bundle adjustment.

### Pi 4 benchmark (offline)

Same sequence on the Pi, after `install.sh`. Copy `data/rgbd_dataset_freiburg1_xyz` into the bundle folder
(`~/pi-bundle` here), then on the Pi:

```bash
mkdir -p ~/tum_out
sudo docker run --rm -v ~/pi-bundle:/stick -v ~/tum_out:/out stella-vslam:arm64 \
  /opt/stella_vslam_examples/build/run_tum_rgbd_slam \
  -v /opt/vocab/orb_vocab.fbow -c /opt/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml \
  -d /stick/rgbd_dataset_freiburg1_xyz --no-sleep --auto-term --viewer none --eval-log-dir /out
sort -n ~/tum_out/track_times.txt | awk '{a[NR]=$1} END {printf "median %.1f ms\n", a[int(NR/2)+1]*1000}'
sudo vcgencmd get_throttled     # 0x0 = no throttling; anything else makes the timing pessimistic
```

Add `--frame-skip 2` to simulate a 15 fps camera. Copy `~/tum_out` back to `data/out_pi` and score it with
`evo_ape` as above.

## 2. Calibrate the ELP (Mac, camera plugged in by USB)

```bash
pip install opencv-python numpy
python3 calibrate_fisheye.py capture --out calib_imgs      # SPACE saves, Q quits; 30-50 images
python3 calibrate_fisheye.py calibrate --images calib_imgs
```

Paste the printed values into `config/elp_fisheye.yaml` and delete the PLACEHOLDER header.
Calibrate at the robot's resolution (640x480 assumed) and don't touch the lens focus afterwards.
`--cols/--rows` (before the subcommand) are the board's inner corners, default 9x6.

## 3. Install on the Pi (console, no SSH)

On the Mac, with an exFAT USB stick:

```bash
./make_usb_bundle.sh /Volumes/STICK                      # Pi has internet
./make_usb_bundle.sh /Volumes/STICK --with-docker-debs   # Pi has no internet and no Docker
```

On the Pi, with a monitor and keyboard:

```bash
cd /media/$USER/STICK/alohamini-slam     # or wherever the stick mounts
sudo bash install.sh                     # add --enable to start SLAM at every boot
```

`install.sh` also runs `pi_inventory.sh` and writes `pi_inventory.txt` back to the stick for issue #41.

## 4. Run on the Pi

Stop the LeRobot host first: only one process can open the camera.

```bash
sudo systemctl start alohamini-slam      # camera + SLAM + viewer
# browse to http://<pi-ip>:3001 from a laptop on the same network
sudo systemctl stop alohamini-slam       # saves the map; wait for it to finish
```

Each run writes to `/opt/alohamini-slam/maps/`:
- `map_<time>.msg`: the map (mapping mode)
- `run_<time>/frame_trajectory.txt`, `keyframe_trajectory.txt`: poses (TUM format)
- `run_<time>/track_times.txt`: per-frame tracking time. This is the Pi 4 benchmark: compare it with 33 ms (30 fps)

Settings are in `/opt/alohamini-slam/slam.env`: camera device, resolution, `MODE=mapping|localization`, `MAP_IN`,
and `VIEWER=none` to save CPU. Logs: `journalctl -u alohamini-slam -f`.

The map is saved only on a clean stop. Pulling power or a dead battery loses that run's map.

## Assumptions

Checked against the Pi inventory of 2026-11-09 and on the Pi on 2026-11-17 (see `../pi-replica/README.md`).

| Assumption | Status | Where to change it |
|---|---|---|
| Pi runs Ubuntu 24.04 arm64 | **Confirmed** (24.04.4, Pi 4, 3.7 GiB RAM, 43 GB free) | `DOCKER_DEB_DIST` in `make_usb_bundle.sh` |
| Docker must be installed by `install.sh` | **Confirmed and done**: `install.sh` installed it on 2026-11-17 | |
| Camera at `/dev/am_camera_forward` | **Wrong**: no `/dev/am_*` udev rules exist. `install.sh` falls back to `/dev/v4l/by-id/*-video-index0`, but only if the camera is plugged in at install time. On the robot Pi it is set by hand to `/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_2020032001-video-index0` | `slam.env` `CAMERA_DEVICE` |
| Camera formats | **Confirmed** with `v4l2-ctl --list-formats-ext` (2026-11-17): YUYV 640x480 at **30 fps only** (also 320x240@30, 800x600@20, 1280x720@10, 1280x960@9). MJPG offers 15 or 30 fps at 320x240 up to 1280x960 | |
| 640x480, YUYV, 30 fps | **Supported**, but the Pi 4 processes about 22-24 fps, so frames are dropped. For an even 15 fps the camera would need MJPG, which costs a decode on the Pi and is untested with `v4l2_camera` | `slam.env`, and `cols/rows/fps` in the config |
| The LeRobot host is not running while SLAM runs | No LeRobot install was found on the Pi, so nothing else should hold the camera | |
| ~170 degree equidistant fisheye, f around 240 px | Unconfirmed until calibrated | `config/elp_fisheye.yaml` |

## Notes

- Monocular: the map has no metric scale. stella_vslam has no IMU input.
- There is no `max_num_keypoints` option in stella_vslam 0.7. To reduce CPU, lower `Feature.num_levels`
  or the resolution, or set `VIEWER=none`. On the Pi 4 this matters: about 44 ms per frame at default settings.
- The Pi's clock and `journalctl` use local time, but `maps/run_*` and `map_*` names are UTC. Use local time with
  `journalctl --since`.
- Docker Desktop on macOS cannot pass USB cameras into containers. Live camera use is Pi-only (or Linux).
- `PUBLISH_TF=false` by default: publishing map -> odom TF needs a camera -> base_link -> odom TF chain on the robot,
  which does not exist yet. Poses are still published on `/run_slam/camera_pose`.
- The offline Docker .debs cover Docker itself. If the Pi is also missing system packages they depend on
  (e.g. `iptables`), `dpkg` will report them and they need internet or more .debs.
