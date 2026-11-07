# Simulation timing tools

Measurements behind `sim_experimental.launch.py` (see "Experimental launch" in
the [humanoid_sim README](../../README.md)). They only observe: no motor is
enabled and no service is called, except where a launch argument such as
`auto_start:=true` asks the launch itself to.

Run from the repository root after building `humanoid_sim`. Every tool sources
`scripts/activate_rosenv.sh` itself.

| Variable | Default | Meaning |
|---|---|---|
| `LAUNCH` | `experimental` | `experimental`: `sim_experimental.launch.py` on CycloneDDS; `sim`: original `sim.launch.py` on Fast DDS. Recorders and probes use the same DDS. |
| `OUT_ROOT` | `local_test/sim_timing` | Result directory (ignored by Git) |
| `DOMAIN` | 81–84, one per tool | ROS domain; keep it at most 101, larger IDs map to Linux ephemeral UDP ports |

Each run writes `<OUT_ROOT>/<tool>_<launch>_<host>_<time>/` with
`environment.txt` (CPU, platform, DDS, `rmem_max`, renderer, Git revision),
`summary.md` and the raw logs. Processes are stopped by domain: everything of
this user whose environment has `ROS_DOMAIN_ID=<domain>`, so do not reuse
these domains for other work while a tool runs.

## startup_trials.sh: startup estops

```bash
simulation/humanoid_sim/tools/timing/startup_trials.sh [N=10]
LAUNCH=sim simulation/humanoid_sim/tools/timing/startup_trials.sh 10
```

Starts the full stack N times (camera + depth + policy, gantry held, 14 s each;
`ARGS="..."` replaces the launch arguments) while `imu_recorder.py` logs every
`/imu/data_raw` stamp. The simulator stamps a sample when it publishes it, so a
stamp gap means the simulator did not publish.

`summary.md` per trial: IMU sample count; *early gaps* before T+1 s (T is
`Simulation ready`; the camera renderer starting, before any check is armed);
*stalls after T+1 s*, which can latch an estop; and the estop or policy fault.
An estop without a simulator stall means the safety node's own process stalled.

Reference (i7-1165G7, native Ubuntu 24.04): Fast DDS about 30 % of startups
estopped, mostly with a ~330 ms stall at T+2.8–3.2 s; CycloneDDS 0 of 25.

## dds_join_repro.sh: minimal Fast DDS stall

```bash
LAUNCH=sim simulation/humanoid_sim/tools/timing/dds_join_repro.sh [N=6]
simulation/humanoid_sim/tools/timing/dds_join_repro.sh 6
```

Only the simulator and the safety supervisor run (`minimal:=true`). 2.5 s in,
`service_clients_node.py` joins: it creates the two simulator service clients
`cpp_control_node` uses and never calls them. Reported are simulator IMU stalls
over 100 ms after that join (E). Reference: Fast DDS 12 of 17 trials stalled
~330 ms at E+1.9–2.3 s; CycloneDDS 0 of 9. The rate varies between batches, so
run 6 or more.

## image_loss.sh: camera frames lost in transport

```bash
simulation/humanoid_sim/tools/timing/image_loss.sh [RUNS=3]
```

Camera and depth on; `image_probe.py` subscribes like `ball_detection_node`
(best effort, depth 1) but takes raw CDR bytes, so Python does not deserialize
the images. Every rendered frame also has a `camera_info` with the same stamp,
which serves as the list of frames sent; a frame sent but not received was lost
in transport. Reference with CycloneDDS: 7–14 % color and 2–9 % depth frames
lost with `net.core.rmem_max` = 208 KB, 0 % with 32 MB.

## camera_matrix.sh: feedback timing with and without the camera

```bash
simulation/humanoid_sim/tools/timing/camera_matrix.sh
WALK=1 simulation/humanoid_sim/tools/timing/camera_matrix.sh   # + two auto_start runs
```

Four held configurations (A no camera, B 640x480 @ 15 Hz, C B + depth,
D 320x240 @ 5 Hz), each `WARMUP_S` (15) then `timing_probe.py` for
`SECONDS_PER_RUN` (30): joint, IMU and policy inter-arrival p99/max, gaps over
50 ms, IMU age, camera rate, physics overruns, real-time factor, CPU per
process. With Fast DDS the probe's own join can cause one ~330 ms stall inside
the window. Walking runs end in falls about half the time with either DDS;
that is a separate, open issue.

## Files

| File | Role |
|---|---|
| `common.sh` | Environment setup, launch selection, start and stop by domain |
| `imu_recorder.py` | Logs receive time and stamp of every IMU sample |
| `startup_report.py` | Summary of `startup_trials.sh` |
| `service_clients_node.py` | Node with two idle simulator service clients |
| `image_probe.py` | Frame rate, latency and transport loss of color and depth |
| `timing_probe.py`, `summarize.py` | Probe and table of `camera_matrix.sh` |
