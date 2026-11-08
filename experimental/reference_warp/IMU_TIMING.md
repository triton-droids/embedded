# Chase IMU timing investigation — 2026-10-31

**Intermittent IMU timeouts are reproduced and are not fully resolved.** This
investigation adds a tested callback-isolation mitigation and diagnostic evidence,
without increasing any safety or control deadline. No hardware was operated.

## Distinguish initiating faults

Previous released chase records show a deliberate `turn_required_not_implemented`
stop, followed by command expiry, physics pause, and then IMU freshness estop.
That final IMU timeout is secondary: frozen physical samples are intentionally
not republished with renewed timestamps. The adapter now preserves its first
latched fault even when simulator expiry/estop follows it; a regression test
verifies this. Read `/experimental/chase_status` alongside Safety.

There is also a genuine startup/scheduling problem while the robot remains on
the gantry, with `control_fault=null` and continuing physics. It must not be
dismissed as the consequence of the turn gate.

## Retained full-stack headless observations

All used EGL, `use_viewer=false`, network DDS (`localhost_only=0`), the complete
WBC/safety/Web composition, isolated domains, and held, disabled leg motors.
Only the camera-ablation record turned camera production off. Raw data and logs
are in `results/`; all completed observations are retained.

| Record stem | Window | Max publisher gap | Safety outcome |
|---|---:|---:|---|
| `imu-baseline-held` | 30 s | 345 ms | Startup: no accepted IMU within 3 s |
| `imu-instrumented-held` | 25 s | 294 ms | No observed Safety trip; WBC/chase separately stopped |
| `imu-isolated-held-1` | 35 s | 294 ms | IMU age 257 ms; dedicated IMU executor alone did not solve it |
| `imu-no-camera-held` | 25 s | 288 ms | IMU age 254 ms; disabling camera did not solve it |
| `imu-callbacks-held` | 22 s | 125 ms | Final Safety clear |
| `imu-gc-held` | 25 s | 129 ms | Final Safety clear |

The last observer had a 331 ms reception gap and a 324 ms maximum observed source
age, while its Safety node remained clear. Different readers can experience
different scheduling/discovery/backlog; an independent subscriber's maximum
gap is not a measurement of Safety's exact receipt gap.

These are held timing observations, **not walking/stability passes**. The two
clear final runs do not cancel the failed isolated-executor run. No causal
claim of a complete fix is supported by this small, non-deterministic cohort.

## Findings and changes

- Callback scheduling gaps reached ~284–294 ms even when the IMU publish call
  was <7 ms and physics-state sampling waits were small. The independent IMU
  executor therefore addresses a real queue-starvation mechanism, but cannot
  eliminate all in-process/GIL/native middleware/OS scheduling stalls.
- A real ROS timer regression blocks an ordinary feedback callback. Before the
  fix, IMU publication did not advance during that blockage; with the new
  `make_executors` separation, the IMU timer continues. Command reception remains
  isolated separately. The generic simulator defaults to its previous executor
  arrangement; this experiment enables `dedicated_imu_executor=true`.
- `/imu/status` now reports maximum callback gap, IMU publish-call duration,
  diagnostic publish duration, and optional GC pause duration in addition to
  the existing publish-gap and sample-wait measurements. `/sim/status` and
  `/can_timing` expose odometry/status/feedback callback maxima. These metrics
  do not renew any physical source stamp.
- The final measured GC maximum was 0.66 ms. This successful run does not
  identify GC as the cause of the failed runs. GC was not disabled.
- Camera ablation did not remove the problem, so offloading/reducing camera
  rendering was not adopted as an asserted cure.
- The exact native/scheduling source of the remaining process-wide startup
  stall is **unidentified**. No safety grace origin or timeout was changed.

Signal-triggered Python thread sampling was rejected: the first sampler fired
before handler registration and terminated that diagnostic simulation; the
second registered sampler later exited with SIGSEGV. These attempts have empty
telemetry (`imu-stack-held*.json`) and retained invalid-attempt logs. They are
not stability evidence. All signal-sampling code was removed; subsequent runs
used only low-overhead duration counters.

## Verification and use

**121 passed, 1 skipped** for the simulator test suite, Safety IMU time-contract
tests, and experimental reference/chase/payload tests. The skipped integration
test is opt-in. The simulator Python package was rebuilt and whitespace/syntax
checks passed. This verifies the implemented seams, not hard-real-time behavior.

Restart the experiment to load the changes. In another terminal:

```bash
source scripts/activate_rosenv.sh
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=0
ros2 topic echo /experimental/chase_status --once
ros2 topic echo /imu/status --once
```

For a bounded recording:

```bash
python experimental/reference_warp/imu_timing_probe.py --duration 30 \
  --output experimental/reference_warp/results/my-imu-timing.json
```

No simulator is intentionally left running after this investigation. Stable
runtime timing, commanded turning, balanced stopping and actual ball arrival
remain separate unresolved requirements; relaxing the 250 ms watchdog would
not establish those capabilities.
