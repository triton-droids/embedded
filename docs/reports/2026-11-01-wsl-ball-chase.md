# WSL ball chase with physical head and GUI

Date: 2026-11-01 (America/Los_Angeles).

## Scope and configuration

Tested on the user's i9-12900HX, WSL Ubuntu 22.04 (24 logical CPUs,
31 GiB available to Linux), ROS 2 Humble and MuJoCo 3.3.7. The full chase
launch retained the physical simulated yaw/pitch head, local GUI, RGB-D camera,
YOLO CPU detector, policy, C++ forwarding, Safety and Web. No hardware was used.

| Component | Measured environment |
|---|---|
| CPU | Intel Core i9-12900HX; WSL exposes 24 logical CPUs |
| Host OS | Windows 11 Home, build 26300 |
| Host memory | 63.7 GiB visible (approximately 64 GiB installed) |
| WSL memory | Approximately 31 GiB visible; 8 GiB swap |
| Linux | Ubuntu 22.04.5 LTS; kernel 6.6.87.2-microsoft-standard-WSL2 |
| ROS / Python | ROS 2 Humble / Python 3.10.12 |
| Runtime | MuJoCo 3.3.7; ONNX Runtime 1.23.2; CPU football inference |
| Image libraries | NumPy 1.26.4; OpenCV 4.11.0 |

The owner subsequently reported that IMU timeouts are resolved on this
device with a sufficiently powerful CPU and the timing optimizations enabled.
This is device-specific operational feedback; no minimum CPU requirement was
benchmarked. The shorter measurements below are reported independently.

The chase launch now exposes `dds_tuning` (default `on`) and `imu_split`
(default `true`). The simulator process environment was inspected in every
tuned trial: asynchronous publication and the installed Fast DDS profile path
were present. GC freeze and the GUI were verified true by live parameters;
Safety's IMU timeout remained 0.25 s. Existing acquisition stamps, latest-frame
inference, 6 Hz camera and 1 s visual TTL were retained. WBC was not enabled:
the shared launch explicitly rejects WBC with vision. Its dedicated command
executor is therefore not claimed as active in these trials.

## Held tracking results

A preliminary 45-second steady observation with tuning off measured 92.2 Hz
IMU, maximum receipt interval 32.5 ms and maximum source age 23.9 ms, with no
control or Safety fault. This was after startup, not a cold-start trial.

A tuned 60-second observation included startup and repeated low/high/low,
left/right and behind-camera ball placements. Both existing runtime smoke
scripts passed. Actual pitch offsets were -0.092 / -0.377 / -0.134 rad;
peak confidence was 0.906 / 0.924 / 0.921. Loss produced zero chase commands
and the ball was reacquired. The horizontal test recorded 60 left and 81 right
tracking status samples, 16 loss/zero samples, and yaw target range
-0.182 to +0.098 rad. Status samples are not distinct detector frames.

After excluding the first eight observer seconds, IMU receipt rate was
90.9 Hz, maximum receipt interval 30.6 ms and maximum source age 22.6 ms.
No control or Safety fault occurred. Last reported real-time factor was 0.89.
There was a 2.25-second startup receipt gap before the control stack started;
the first observer sample was already 2.13 seconds old. This startup artifact
is retained in raw results and is not included in steady-state statistics.
The two observations differ in workload and startup coverage, so they do not
establish a causal performance gain from DDS tuning.

## Release trials

Two independent tuned launches used `auto_start:=true`, each with a requested
60-second observation including startup. No reset or re-enable followed a fault.

| Trial | Release-to-first-estop | First cause |
|---|---:|---|
| 1 | 1.90 s | Fall detection, roll -36.2 degrees, pitch -45.6 degrees |
| 2 | 7.74 s | Fall detection, roll -11.0 degrees, pitch -40.7 degrees |

Both first faults were falls, not IMU freshness. Estop then paused physics;
later absent or stale sensor data cannot be interpreted as the initiating
failure. Safety heartbeat advisory warnings were also observed and remain
unresolved. These launches do not establish stable walking. Auto-start runs
the bundled reference motion; its policy does not consume chase velocity.

## Reproduction and evidence

```bash
export ROS_DISTRO=humble
source scripts/activate_rosenv.sh
ros2 launch humanoid_ball_chase ball_chase.launch.py dds_tuning:=on
# Separate release trial:
ros2 launch humanoid_ball_chase ball_chase.launch.py dds_tuning:=on auto_start:=true
```

Local raw observer samples, summaries, process-environment checks, launch logs,
and smoke-test output are under ignored `local_test/chase_device/`. The first
observer attempt failed to serialize Humble's byte-valued diagnostic level;
its measurements were discarded and the observation repeated after correcting
serialization. An initial launch also used the old installed launch copy;
the package was rebuilt before all trials labeled `tuned_*`.

Package build, launch Python compilation and `git diff --check` passed.
The selected simulator, ball-chase and Safety regression suite reported
179 passed and 1 skipped (opt-in integration test). It emitted one existing
SciPy/NumPy compatibility warning. The short campaign did not reproduce a
primary IMU timeout; longer-term resolution is the owner's subsequent report,
not a guarantee inferred from these limited trials.
