# Exploratory results — 2026-10-31

The reference modification reaches actual policy inference and can run through
the existing WBC/C++/safety simulation chain. It has **not** established reliable
step-length control, commanded turning, or an improvement over the original.

Only files under `experimental/reference_warp/` were added for this experiment.
The original ONNX, registry and production code were not edited. Generated
variants are ignored artifacts. They retain the trained 50 Hz contract and all
original safety deadlines. No hardware was operated.

## Modification and checks

- Identity, 0.95 and 1.05 sagittal reference-excursion variants; maximum modified
  reference-angle difference is 0.036214 rad (2.075 degrees).
- All neural weights and non-reference initializers are byte-identical.
- Source ONNX SHA256 remains
  `90fc691abbf43b89a55956493da6ab19c59105f9167f29f8f93fc317b68c9309`.
- Source registry SHA256 remains
  `e3bb3842ba4f97cbadc11f2261677264828f36ecf26cc62b25b9a3fbc5c3ca08`.
- Eight tests passed, including actual ONNX export/reload/inference, unchanged
  startup response, exact identity, excluded-axis preservation, ramp velocity
  consistency, scale validation, and prevention of output outside the experiment.
- With identical synthetic feedback (original reference q/dq, zero gyro, down
  gravity, zero previous action), frame 0 produces exactly the original output.
  At frame 100, maximum target-angle change is 0.757 degrees for 0.95 and
  0.740 degrees for 1.05. These are offline inference checks, not walking metrics.

## Normal GUI trials

Six recorded trials used `rl_wbc_sim.launch.py`, `policy_impedance`, auto-start,
an isolated ROS domain (181), Web port 8181 and an actual new MuJoCo GUI window.
Every record is retained in [the cohort directory](results/20261031-184251/).
The earlier identity attempt in `results/20261031-184209/` additionally had an
instrumentation error and is excluded from these six, not silently replaced.

| Reference scale | Repeat | Result | Approx. time from observed release to first reported stop |
|---|---|---|---|
| 1.00 | 1 | Censored stop: IMU freshness | 0.25 s |
| 0.95 | 1 | Not released: safety/enable admission | — |
| 1.05 | 1 | Full 10 s observation, no observed fall | No stop |
| 1.00 | 2 | Censored stop: motor command expiry | 3.15 s |
| 0.95 | 2 | Censored stop: IMU freshness | 4.46 s |
| 1.05 | 2 | Censored stop: disabled motor, input expiry and safety stop | 0.49 s |

These stop times are approximate because the monitor receives periodic statuses.
The JSON `exposure_wall_s` field is the whole post-release observation interval,
including any paused tail; it must not be confused with active walking exposure.
No confirmed falls occurred, but the five interrupted/not-released observations
are **not** stability passes. Concurrent camera simulation was also running
during this cohort; timing failures cannot be assigned to the reference change.

The uninterrupted 1.05 trial:

- Base displacement in world XY: **(+1.249, +4.643) m** over ~10 wall seconds.
  Robot toes initially face world +Y.
- Lowest base Z: 0.676 m; maximum absolute roll/pitch: 11.59 / 6.61 degrees.
- Policy still running at the end, two reference-loop wraps, no policy fault,
  no simulator control fault, and final safety status clear.
- Yaw drifted from approximately 0 to **-26.15 degrees** without a turn command.
  This is uncommanded drift, not a demonstrated turn controller.

There is no uninterrupted identity control with equal exposure in this small
cohort. Therefore it is invalid to conclude that +5% reference excursion causes
+5% footstep length, increases speed, or improves stability. Physical footstep
length was not measured. The experiment confirms feasibility of changing the
reference and obtaining a walking rollout, with reproducibility still limited
by the existing timing/stop behavior.

## Next experimental stage

First establish repeatable original/modified trials with equal active exposure,
record foot touchdown positions and actual stride lengths, and separate startup
timing failures from falls. Only then assess a calibrated excursion-to-speed
mapping. Turning requires its own explicit target and foot-trajectory experiment;
the observed yaw drift must not be repurposed as a turning command.
