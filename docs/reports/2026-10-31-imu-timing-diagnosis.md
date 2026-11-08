# IMU timing diagnosis and simulation mitigations

Date: 2026-10-31 (America/Los_Angeles).

## Result

Several timing and workload issues were corrected, but the intermittent failure is **not fully resolved**. Two final ordinary GUI trials completed their full 45-second launch observation windows, including approximately 37 seconds after gantry release, without a control fault. Another trial tripped IMU freshness before ENABLE. A controlled-load trial later stopped on actual tilt rather than an initial IMU timeout. These results do not establish reliable walking under every load or fix the user's separate Windows/WSL machine.

All launch tests used the full `rl_wbc_sim.launch.py` chain: policy, WBC, C++ command forwarding, simulation backend, Safety, and Web. Body IMU remained 100 Hz, physics 5 ms, policy 50 Hz, WBC 100 Hz, and C++ output 200 Hz. Safety's IMU timeout remained enabled at 250 ms. Source freshness, explicit command expiry, fall checks, and explicit re-enable requirements were retained. Viewer state synchronization remained at the user's requested 5 Hz; this does not cap the passive window's native redraw loop.

The host was the existing Ubuntu simulation workstation, not the user's i9 WSL installation. No hardware or CAN commissioning was performed.

## Observed mechanisms

1. **A long publish call can starve safety input processing.** In `fixed_stressed_2/timing-1621631.json`, `/safety/status` publish took 317.48 ms and the Safety evaluation callback gap reached 339.63 ms. The supervisor then reported IMU age 0.341 s. This identifies the publish path as a blocking location; elapsed time also includes native conversion, scheduling, and possible middleware waits, so it does not prove Fast DDS alone consumed the time.
2. **IMU timeout can be a consequence of command loss.** In the first pressure reproduction, the backend logged command expiry and physics pause at `1793462842.417892`, WBC input timeout at `.504059`, policy JointState timeout at `.552031`, and Safety IMU timeout at `.717131`. The simulator intentionally stops IMU and joint publication after a control fault. Later increasing ages must not be mistaken for the initial cause. Log timestamps are emission times; Safety publishes its estop before its fatal diagnostic log.
3. **The backend can lag behind another reader of the same command stream.** In `coherent_gui_2`, the backend rejected a command aged 102.2 ms at `1793470170.468310`, while the independent observer was already receiving newer source stamps. This is consistent with reader/executor backlog rather than every publisher having stopped. It does not by itself distinguish all middleware and callback-queue contributions.
4. **Callback timestamps were not acquisition timestamps.** Body IMU and joints stamped before waiting for the physical-state lock, while odometry stamped separately. Scheduler delay could therefore appear as source delay or input skew even when the copied physical state was newer. The fix records source time at the physical step, and never renews it merely by reading or republishing a snapshot.
5. **There is also a distinct fall mode.** Several trials tripped `fall_detection` while the pre-fault IMU stream remained near 100 Hz. Removing IMU checks would not resolve that balance/timing problem.

An observer's receipt gap is not necessarily the gap experienced by Safety or policy. In particular, the first successful final trial had a 270 ms best-effort observer gap without a controller fault. Average Hz alone cannot identify a stale-data cause.

## Implemented changes

- WBC simulation creates `sim_motor_command_rx` in the simulator process with a dedicated single-threaded executor. Reliable command and estop reception no longer share the sensor timer queue. Shared motor and physics state still uses `world.lock`; lifecycle queue depth and ENABLE/DISABLE semantics are retained. Set `dedicated_command_executor:=false` for an A/B comparison. PD/minimal retain the original executor arrangement.
- Physical steps record wall-clock acquisition timestamps and monotonic sample time. Body IMU, joint feedback, odometry, and foot feedback preserve this provenance. Pose and foot contacts are copied atomically from one physical step. Repeated snapshot reads cannot make old data fresh; backend feedback age uses acquisition time.
- The simulation launch applies asynchronous Fast DDS publication to the entire stack, including Safety, policy, and Web. Previously only simulator/WBC/C++ were configured this way.
- Simulation diagnostics `/safety/status` use best-effort depth 1. Existing C++ and Web consumers already request best-effort. The standalone safety launch defaults to reliable diagnostics. `/safety/estop` always remains reliable and transient-local.
- Web consumes a separate 10 Hz `/sim/monitor/joint_states` stream. Control feedback stays at its original rate on `/joint_states`. Monitoring preserves the acquisition stamp and stops on a physics fault.
- Autostart exits after success or failure instead of continuing to deserialize command traffic. It waits up to 200 ms for reliable command acknowledgements before destruction. WBC startup waiting is 10 seconds on the gantry; runtime deadlines and admission checks are unchanged.
- `/imu/status` exposes `last_publish_age_s`, `max_publish_gap_s`, `max_sample_wait_s`, and `control_fault`.

These are mitigations and correctness improvements, not a hard-real-time guarantee. Asynchronous DDS writes can still block under resource pressure; see [eProsima's publication-mode discussion](https://www.eprosima.com/developer-resources/performance/dds-asynchronous-vs-synchronous-publishing).

## Validation

Regression command, run in isolated DDS domains with writable temporary ROS logs:

```bash
RUN_ROS_INTEGRATION=1 ROS_LOG_DIR=/tmp/imu_diag_final_tests \
ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 python -m pytest \
  simulation/humanoid_sim/test \
  humanoid_control/wbc_control_node/test/test_bridge.py \
  humanoid_control/motor_control_hybrid/test/test_tracking_node.py \
  experimental/test_wbc_model.py \
  humanoid_control/humanoid_safety/test/test_checks.py \
  humanoid_control/humanoid_safety/test/test_imu_time_contract.py \
  humanoid_control/humanoid_safety/test/test_motor_monitor.py \
  humanoid_control/humanoid_safety/test/test_status_qos.py -q
```

**160 passed**, including the opt-in DDS test for frozen ROS time, replayed IMU rejection, and retained estop delivery. New tests cover acquisition-time preservation, frozen-state timestamps, monitoring provenance, terminal autostart cleanup, unchanged estop QoS, and command admission while feedback work is blocked. The latter uses isolated domain 193 and real separate executors. `git diff --check` passed.

Final GUI launch trials used isolated domain 187 and Web port 18187. Each requested 45 seconds from launch, with startup time included:

| Trial | Extra CPU load | Released | Post-release IMU observation | Outcome |
|---|---|---|---|---|
| `dedicated_gui_1` | None | Yes | 37.00 s, 99.43 Hz | No control fault; observer max gap 269.92 ms |
| `dedicated_gui_2` | None | Yes | 36.71 s, 99.72 Hz | No control fault; observer max gap 26.57 ms |
| `dedicated_gui_3` | None | No | Not applicable | Startup IMU freshness trip, age 0.340 s; excluded from walking success |
| `dedicated_stressed` | Two bounded busy-loop processes | Yes | 33.53 s before stop, 100.00 Hz | Tilt trip, pitch -47.7 degrees; not an initial IMU/command-expiry failure |
| `gc_diagnostic` | None; GC timing hook | Yes | 36.79 s, 99.65 Hz | No control fault; observer max gap 25.90 ms |

The load processes alternated approximately 20 ms busy work and 5 ms sleep and were bounded to 50 seconds. This is an artificial contention test, not a measurement of ordinary desktop load. Faulted trials continued observing until their requested window ended; stopped IMU publication limits the reported pre-fault IMU span.

## Rejected or inconclusive experiments

- A generic two-thread `MultiThreadedExecutor` reduced observed IMU rate to about 90 Hz and produced 24 physics overruns in one trial. It was removed; the shipped design uses two targeted single-thread executors in WBC mode.
- UDP-only Fast DDS did not eliminate command expiry. A signal-instrumented UDP run also crashed the simulator; that run is not valid stability evidence, and no UDP-only default was adopted.
- A temporary, uninstalled Cyclone DDS overlay did not yield a working observation chain and had teardown problems. This does not support recommending a middleware switch. System package installation was not changed.
- Native `py-spy` profiling perturbed deadlines and the profiler itself segfaulted. No native stack attribution was accepted from that experiment.
- One interrupted run experienced an approximately hour-long wall-time discontinuity. It was excluded from ordinary timing comparisons.
- A GC-only hook measured a maximum collection around 21.74 ms in its successful run. No evidence justified disabling cyclic GC, so GC behavior was not changed.

Two authorized DeepSeek read-only workers completed code audits (exit status 0). Their hypotheses were reviewed against live measurements; speculative suggestions were not treated as verified causes.

## Remaining work and artifact handling

### Post-pull pacing regression

After updating to `909d0f9`, an isolated test of the real `SimWorld._run`
loop reproduced an additional slowdown introduced by the upstream no-catch-up
change. With a 5 ms physics period and a 6 ms step, the scheduler waited another
5 ms after the step, producing an 11 ms start-to-start interval (90.91 Hz).
This establishes a scheduler regression under overload, but does not establish
that every slowdown on the user's host has the same cause.

The correction preserves ordinary absolute-deadline pacing. When a deadline is
missed, it discards historical deadlines and schedules the next step no earlier
than the current step's start plus one period. If the step already consumed
that period, it yields with `sleep(0)` rather than waiting another full period.
It neither replays a backlog nor skips MuJoCo integration steps. Five regression
cases cover normal costs, overloaded costs, and delayed scheduler wake-up;
three failed before the correction and all five pass afterward.

A three-second benchmark using the actual MuJoCo world, held on the gantry,
measured the following. The injected delay is an artificial 6 ms sleep inside
each measured step, not a claim about ordinary CPU computation cost.

| Condition | Upstream pacing | Corrected pacing |
|---|---:|---:|
| No injected delay | 200.26 Hz; real-time factor 1.001 | 200.28 Hz; real-time factor 1.001 |
| Additional 6 ms delay per step | 85.64 Hz; real-time factor 0.428 | 152.12 Hz; real-time factor 0.761 |

The complete previously selected regression suite, including opt-in DDS
integration, now passes **165 tests**. These benchmark results do not prove
walking stability or hard real-time scheduling on the failing host.

A subsequent full GUI `ros2 launch wbc_control_node rl_wbc_sim.launch.py`
trial, observed for 32 seconds with automatic start, released the gantry but
tripped IMU freshness about 1.5 seconds later (reported source age 0.344 s).
The observer received 640 IMU samples at 93.14 Hz across the active span, with
a maximum receipt gap of 332.72 ms. Physics was paused after estop. This is a
failed full-stack trial, not evidence that the remaining IMU stall is fixed;
the overload-pacing correction is the narrower verified improvement.

Intermittent cold-start IMU trips and balance under contention remain open. The changes reduce avoidable work, isolate command admission, and correct source-time reporting, but the evidence does not justify claiming the entire issue is fixed or raising safety timeouts. The next investigation should correlate the first consumer gap, accepted/rejected source counts, publisher duration, and native scheduling on the actual failing host.

### Optional remote rendering

Added a read-only TCP state stream behind `remote_viewer:=true`, with loopback
binding by default and a configurable 15 Hz sample rate. Network IO and initial
compiled-model serialization run in a separate process. A bounded sampler skips
busy physics locks, and a one-element queue drops old snapshots. Model, meshes
and textures transfer once per connection; the independent Python client needs
MuJoCo and NumPy but no ROS or local model assets. The client reconstructs the
scene without advancing physics and has no control channel to the simulator.
Exact MuJoCo versions must match. Direct LAN use is unauthenticated; an SSH
tunnel is documented for the default loopback binding.

Ten real TCP/protocol tests cover transferred geometry/state, reconnect,
absent/idle clients, occupied ports, version mismatch, bounded packets,
non-finite states and invalid rates. The selected full regression suite now
passes **175 tests**. A complete WBC ROS launch with the local viewer disabled
and the stream enabled was tested with separate client processes: 20 headless
frames, 15 GUI frames, then 20 headless frames after reconnect, all successful.
This was local TCP testing on Ubuntu, not cross-host or Windows validation.
The full launch still reported an IMU freshness trip (0.341 s) and therefore
does not demonstrate resolved startup or walking stability. No controlled CPU
reduction measurement was made. Remote viewer process logs and the launch
check remain ignored under `test_results/remote_viewer_20261031/`.

Raw probes, timing instrumentation, delegated audit transcripts, temporary middleware libraries, and all process files remain ignored under `test_results/imu_timing_20261031/`. The historical 2026-10-30 60-trial comparison report was not rewritten. Changes are not committed or pushed. Diagnostic processes created for this investigation were stopped.
