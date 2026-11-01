# Combined chase experiment — 2026-10-31

Historical PD configuration results. The current experimental launch now uses
WBC; see [WBC_CHASE_RESULTS.md](WBC_CHASE_RESULTS.md). These earlier trials are
retained without reclassifying them as WBC evidence.

The real RGB-D chase request now drives an experimental runtime reference warp
through the original tracking policy, C++ scheduler, safety and simulated motor
backend. This establishes the interface connection, **not reliable autonomous
ball pursuit**. Neither released trial reached the 1 m stop distance: both
stopped when the required turn exceeded the deliberately limited heading corridor.

All additions are under `experimental/reference_warp/`. No production source,
ONNX weights, motor registry or safety deadline was changed. The combined launch
uses the existing PD/C++ route; the production restriction on WBC plus vision
remains intact. The prior WBC-only walking experiment is a separate configuration.

## Validation

- **27 tests passed**, including the static export tests and real ROS policy
  decoding/inference, chase-to-reference mapping, scale derivative, stamped
  command validation, no-command startup, zero/turn/timeout invalidation,
  time budget, loss latching, and calibration-before-arming regression.
- The absolute-path launch imports and runs with the installed ROS workspace.
- A new MuJoCo GUI window was observed for the test launches.
- Source ONNX and registry SHA256 remain the values in `RESULTS.md`.

## Retained trials

| Record in `results/` | Configuration | Outcome |
|---|---|---|
| `chase-held.json` | Held, original offset ball | Positive chase, 105% reference, matching C++ forwarding; ball occlusion latches stop |
| `chase-release.json` | Intended release, old `auto_start` flag | Not released: nested launch overrode flag; retained as a startup configuration failure |
| `chase-live.json` | Released, original offset ball | Forward displacement ~0.209 m, lateral ~0.070 m; stops for unsupported turn demand |
| `chase-centered.json` | Centered ball, first startup logic | Not released: transient input during calibration latched the old arming logic; no policy motor commands |
| `chase-centered-v2.json` | Released, centered ball, corrected startup | Forward displacement ~0.331 m, lateral ~0.046 m; stops for unsupported turn demand |
| `chase-distance.json` | Held, centered ball moved near/far/behind | Actual depth-derived command changes reference scale, then loss stops refreshing commands |

Each JSON has an accompanying `.log`. Earlier failures were preserved, not
replaced. GUI sensor/telemetry histories are wall-time observations and include
the paused tail after a stop. Released displacement is measured from first
received held odometry to final paused odometry; it is not a full-stride or
speed calibration. The released runs were censored by the experiment's turning
gate and cannot establish stable long-duration locomotion.

A subsequent default held preview is retained as `results/chase-preview.log`.
It was not released and later hit the existing IMU freshness estop (0.330 s
against 0.250 s). It was shut down; no experiment simulation is intentionally
left running. This additional diagnostic preview has no full telemetry JSON and
is not counted as a successful trial.

The old ambiguous `auto_start` argument was replaced by `start_chase`. Initial
input loss now only waits until the parent actually produces its first policy
command; after that, loss still latches a stop with the same strict deadlines.

## Distance and loss test

In the final held test, all received simulator statuses confirmed the base
remained held. The input was real camera RGB + aligned `16UC1` depth, with the
existing YOLO detector and image-time camera-to-base transform.

| Ball placement | Measured forward component in base frame | Chase forward request | Applied reference scale |
|---|---:|---:|---:|
| World Y = 1.8 m | ~1.712 m | 0.200 | 1.050 |
| World Y = 1.2 m | ~1.115 m | ~0.1155 | ~1.00775 |
| World Y restored to 1.8 m | ~1.712 m | 0.200 | 1.050 |

These requests have nominal m/s units, but the heuristic does **not** make the
tracking policy achieve those speeds. Only the reference excursion is changed.
The angular request is a gate, not a turn actuator. No target ground truth or
simulator ball position was used to calculate the chase command; placement was
only a test stimulus.

The test recorded 752 C++ outputs with stamps matching captured policy input
stamps. At monitor time 18.009 s the ball was placed behind the robot; the first
observed latched stop was at 18.168 s, and the last policy input was at 18.149 s.
Final held-test safety status remained clear. This is a software/held-simulation
loss test, not proof of a balanced stopping transition while walking.

## Stopping behavior and limits

The released tests logged C++-accepted ENABLE, verified enabled feedback and
gantry release. The experimental adapter then stopped on excessive turn demand;
the existing backend command-loss path paused physics. The paused IMU stream
subsequently triggered the unchanged freshness supervisor. This subsequent
estop is not classified as a fall or successful standing stop.

Commanded turning, calibrated speed/stride, balanced stopping and A* path
following remain unimplemented. The current experiment supports a bounded
forward-approach attempt and refuses unsupported navigation requests. It never
teleports the robot, applies chase forces or catches it with the gantry.
