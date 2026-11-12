# VIO and Isaac ROS detection trials (Jetson, 2026-11-11/12)

Trial scripts, measured results and videos for two features on
`sim-experimental-cyclonedds`:

- **Localization without simulator odometry**: body IMU (simulated BNO085 game
  rotation vector) plus cuVSLAM passive stereo VO, fused by
  `humanoid_localization/vio_localizer` into `/localization/odom`
  (package README: `humanoid_control/humanoid_localization/README.md`).
- **Isaac ROS YOLO as the chase detector**: the td-perception YOLO11n engine in
  the Isaac ROS NITROS pipeline (`scripts/run_isaac_yolo.sh`), with
  `ball_chase_vision detector:=isaac` choosing the chased class by id.

Everything here is evaluation tooling: the scripts read simulator ground truth to
move the ball or score the result; the robot stack never does.

```
scripts/   trial, recording and rendering scripts (run on the Jetson)
results/   per-sample localization errors and the detector comparison
videos/    rendered recordings (not in git: render them with the scripts)
```

## Setup the scripts assume

- Jetson Orin Nano 8 GB, JetPack 6.2.1 (L4T R36.4.7), ROS 2 Humble.
- Checkout at `~/Documents/Github/embedded-vio`, built with
  `colcon build --symlink-install --packages-up-to humanoid_ball_chase walking_controller humanoid_localization --cmake-args -DTRACK_ONLY=ON`.
- Isaac ROS image `humanoid_cuvslam:3.2` (`scripts/run_cuvslam.sh build`), used
  for both cuVSLAM and the YOLO pipeline.
- Scripts copied to `~/vio_runs/`; each run writes `~/vio_runs/<NAME>/`.
- ROS_DOMAIN_ID 57 (VIO trials) or 58 (detector trials), CycloneDDS, localhost only.
- The TensorRT engine `simulation/humanoid_ball_chase/assets/td-perception-fp16.engine`
  built on that Jetson (host and container both have TensorRT 10.3.0.30).

Paths are hard-coded for that layout; adjust `R=` at the top of each shell script.

## Scripts

| Script | What it does |
|---|---|
| `trial.sh NAME EVAL_S [launch args]` | VIO trial: starts cuVSLAM, then the hard-scene chase with `enable_vio:=true`, runs `localization_eval` for `EVAL_S` s. `ROUTE_LEGS=16` adds `route_driver` (long route), `RECORD_S=290` adds `record_route.py`. Writes `errors.csv`, `eval.json`, status snapshots and logs. |
| `vio_chase.sh NAME SECONDS LEGS` | VIO + Isaac YOLO chase along a route, recorded with `record_chase.py` (no evaluator, to keep the load down). Used for `videos/vio_isaac_chase.mp4`. |
| `isaac_chase.sh NAME CLASS SECONDS` | Chase with `vision_detector:=isaac vision_target_class:=CLASS`. `BALL_XY=0.8,1.0` moves the ball after release, `RECORD=1` records with `record_chase.py`. Used for `videos/isaac_target_class.mp4`. |
| `yolo_trial.sh NAME SECONDS` | Runs the Isaac YOLO pipeline next to the built-in Python/TensorRT detector and compares them with `compare_yolo.py`. |
| `compare_yolo.py DURATION [W H]` | Pairs `/isaac_yolo/detections` with `/vision/target_center` by image stamp: ball agreement, pixel difference (both padding conventions), classes seen, latency from the image stamp. |
| `record_route.py DURATION OUT.npz` | Records qpos/mocap at 15 Hz from the simulator's shared-memory state (`state_shm`, no ROS) and copies the model. |
| `render_route.py REC.npz ERRORS.csv OUT.mp4` | Video for a VIO route: chase view, stereo-left inset, top-down truth vs `/localization/odom` from the evaluator CSV. |
| `record_chase.py DURATION OUT.npz` | Shared-memory state at 15 Hz plus low-rate ROS: Isaac detections, chased pixel, chased 3D point, `/localization/odom` (20 Hz). Needs `share_state:=true` or stereo. |
| `render_chase.py OUT.mp4 REC.npz "TITLE" [...]` | Video per recording (joined): chase view; head camera with the Isaac boxes and the chased target; top-down with truth (white path, green ball), the robot's own estimate (orange) and where it believes the target is (yellow, from its own pose + head kinematics + the detection's depth). Skips frames from a stale state file left by an earlier run. |
| `probe_start.py` | Offline (no ROS): stand, then start walking; times every `controller.step` and profiles slow cycles. Used to rule out the controller itself for the start-up faults (single-threaded BLAS: max 4.9 ms on the Jetson). |

## Results

### Localization error (`results/t*_errors.csv`)

Columns: `stamp_s, x, y, true_x, true_y, position_error_m, yaw_error_rad`
(from `localization_eval`, estimate paired with ground truth within 20 ms).
Hard scene, 16-leg route (`route_driver`), path measured at 1 Hz.

| Run | IMU heading error | Path | Final error | Notes |
|---|---|---|---|---|
| t5 | none | 13.6 m | 0.29 m (2.1 %) | old Jetson; errors grow in the untextured south-west corner |
| t6 | ±3° sine, 60 s | 4.8 m | 0.15 m | aborted at 77 s (command expiry under load) |
| t7 | ±3° sine, 60 s | 1.1 m | 0.06 m | aborted at 20 s (same) |
| t8 | ±3° sine, 60 s | 12.7 m | 0.31 m (2.4 %) | before the stereo split |
| t9 | ±3° sine, 60 s | 14.0 m | 0.22 m (1.6 %) | stereo rendered out of process; localizer started 4 s late (5 cm, −1.8° start offset) |
| t10 | ±3° sine, 60 s | 13.9 m | 0.36 m (2.6 %) | localizer initialized before the robot moved |
| t15 | ±3° sine, 60 s | 13.9 m | 0.31 m (2.2 %) | the run in `videos/vio_long_route.mp4` |

The heading error of the IMU passes straight into the estimate (VO rotation is
not fused yet); a sine error mostly cancels over a loop, a one-way drift would not.

### Isaac YOLO vs built-in detector (`results/y4_isaac_vs_python.json`)

60 s, hard scene, 6 Hz camera, same td-perception engine: ball found in 350 / 342
frames, both in 342; ball centers differ by 0.64 px median (Isaac pads centered,
like Ultralytics); latency from the image stamp 117 / 110 ms median (mostly the
simulated camera itself). Isaac also reports robots, line intersections,
goalposts and the penalty mark. No CPU saving was measurable in simulation
(images reach the container over DDS; NITROS zero-copy needs the camera driver in
the same container).

### Chased class (`isaac_chase.sh`, ball moved to (0.8, 1.0))

| Run | Class | End position | Result |
|---|---|---|---|
| c0b | 0 ball | (0.44, 0.59) | stops ~0.55 m from the ball |
| c2b | 2 robot, best score per frame | (0.31, 0.82) | target switches between robots, no clear path |
| c2c | 2 robot, lock on nearest | (−0.11, 1.51) | walks straight to the robot row |

## Videos

The MP4s are kept out of git (`.gitignore`); re-render them from a recording with
`render_route.py` / `render_chase.py`.

| File | Content |
|---|---|
| `videos/vio_long_route.mp4` | t15: 16-leg route with IMU heading error; chase view, stereo-left inset, top-down truth (white) vs `/localization/odom` (orange). 2x. |
| `videos/isaac_target_class.mp4` | Same scene and detector, `vision_target_class:=2` (lock on nearest robot) then `:=0` (ball); head camera with Isaac boxes. 2x. |
| `videos/vio_isaac_chase.mp4` | VIO + Isaac YOLO ball chase along a route; top-down shows the robot's own pose and where it believes the ball is, with live errors (own position median 2.7 cm, max 31 cm over 5 m). Ends when the robot falls while turning (see below). 2x. |

Head-camera panels are re-rendered from the recorded state, so they lack the
hard scene's image degradation; the boxes are what Isaac detected on the
degraded images.

## Known issues

- **Load on the Orin Nano.** Simulation, stereo rendering, cuVSLAM, the Isaac YOLO
  container and the walking controller together leave little CPU. Symptoms:
  walking-controller cycles over 30 ms, command expiry, physics pause, then an
  IMU-freshness estop. The trial scripts renice the evaluation tools to 19 and
  avoid 200 Hz Python subscriptions; with cuVSLAM and YOLO both running the robot
  still fell twice while turning (`vio_isaac_chase.mp4`).
- **`Physics paused` right after `signal_handler(SIGINT)`** in `sim.log` is the
  script tearing the launch down, not a fault.
- **Start order.** The localizer takes the start pose at its first VO frame:
  start cuVSLAM before the robot moves (`route_driver` parks the ball out of view
  until `/localization/status` reports `initialized`).
- Raw recordings (`*.npz` + 34 MB `*.mjb`) and logs stay on the Jetson under
  `~/vio_runs/`.
