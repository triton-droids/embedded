# Headless MuJoCo comparison: WBC and policy PD

## Result

The current WBC launch configuration was more stable than the default policy-PD
configuration in this campaign: **30/30** versus **10/30** valid trials remained
stable for 30 seconds after gantry release.

This compares two complete launch configurations. It does **not** isolate the
causal effect of the WBC optimizer: feedback/forwarding rates, reference playback,
release synchronization, and scene configuration also differ.

| Outcome | Policy PD, no WBC | WBC, `policy_impedance` |
|---|---:|---:|
| Valid trials | 30 | 30 |
| Stable for the full 30-second window | 10/30 (33.3%) | 30/30 (100%) |
| Geometric falls | 17 | 0 |
| Safety fall-detection trips | 20 | 0 |
| Geometric fall or safety fall trip, deduplicated | 20 | 0 |
| Other control/safety failures | 0 | 0 |
| Full-chain pass after evidence review | 10/30 | 30/30 |
| Original probe pass, including discovery false negatives | 10/30 | 28/30 |
| Excluded attempts, replaced with fresh launches | 4 | 10 |
| Total campaign launches | 34 | 40 |
| Walking criterion satisfied | Not directly evaluable | 30/30 |

Geometric falls and safety fall trips can occur in the same trial; those rows
must not be added. Stability requires no geometric fall, post-release estop,
policy/WBC fault, or simulator control fault. A full-chain pass additionally
requires fresh C++ command forwarding, IMU/joint traffic, safety diagnostics,
Web HTTP availability, and evidence of all expected nodes.

## Execution and scope

- **Executor:** a DeepSeek Harness subagent; Codex reviewed the probe and audited
  the recorded results.
- **Campaign time:** October 30, 2026, 21:54:45–22:51:26, America/Los_Angeles
  (UTC−07:00).
- **Trials:** independent restarts, with valid trials interleaved PD then WBC.
  Excluded attempts were retried before advancing to the other configuration.
- **Observation:** 30 wall-clock seconds after the first observed gantry release;
  startup had a separate 25-second budget.
- **Execution:** actual `ros2 launch`, headless, with the complete default control
  and safety stack. No direct policy-to-backend shortcut was used.
- **Environment:** ROS 2 Jazzy, MuJoCo 3.3.7, 5 ms physics steps, flat ground,
  configured zero IMU noise, and no injected pushes or initial-state randomization.
- **Source baseline:** HEAD `2ee73e5ef0929f2c31633056824c46c5548034c4`, including
  the then-uncommitted WBC/simulator changes. Recorded source hashes were unchanged
  throughout testing; controller and safety settings were not edited for the run.
- **Policy:** the same bundled ONNX export in both groups, SHA256
  `90fc691abbf43b89a55956493da6ab19c59105f9167f29f8f93fc317b68c9309`.

The launch commands, after activating the workspace, were:

```bash
source scripts/activate_rosenv.sh

# Policy PD
ros2 launch humanoid_sim sim.launch.py \
  use_viewer:=false auto_start:=true minimal:=false enable_wbc:=false \
  ros_domain_id:=181 web_port:=18181

# WBC
ros2 launch wbc_control_node rl_wbc_sim.launch.py \
  use_viewer:=false auto_start:=true \
  ros_domain_id:=181 web_port:=18181
```

DDS discovery was confined to localhost in domain 181; the Web monitor used
`127.0.0.1:18181`.

The command paths were:

```text
PD:  simulated IMU/joints -> tracking_policy_node
     -> cpp_control_node -> /motor_commands -> simulated motor backend

WBC: simulated IMU/joints -> tracking_policy_node -> wbc_control_node
     -> cpp_control_node -> /motor_commands -> simulated motor backend
```

Both configurations retained `safety_node`, `motor_web_node`, `sim_autostart`,
`sim_body_imu`, `sim_motor_backend`, and `mujoco_sim`. PD also retained the
simulated head node. RViz and camera/vision were disabled optional components;
`minimal` mode was not used. The probe recorded commands, IMU/joint feedback,
safety diagnostics and estop, policy/simulator status, and base ground truth.
WBC trials additionally recorded foot heights/normal forces and QP status.

All 60 valid trials completed their full post-release observation window,
including trials whose physics froze after a fault. Each valid trial had at
least 50 fresh C++ source-timestamp matches and a reachable Web HTTP endpoint.

## Exclusion policy

At the user's request, unreleased startups and primary IMU/input-freshness
problems were recorded separately and replaced by independent restarts. A fall
or primary control/safety fault followed by an IMU timeout remained a counted
failure. Safety thresholds were not relaxed, and no trial reset estop or
re-enabled motors after a fault.

| Exclusion | PD | WBC |
|---|---:|---:|
| Gantry not released within the startup budget | 4 | 8 |
| Primary IMU fault before a fall/control consequence | 0 | 2 |
| Total | 4 | 10 |

Excluded PD attempts: `pd-010`, `pd-012`, `pd-025`, `pd-027`.

Excluded WBC attempts: `wbc-006`, `wbc-008`, `wbc-010`, `wbc-011`, `wbc-013`,
`wbc-019`, `wbc-020`, `wbc-023`, `wbc-026`, `wbc-038`.
`wbc-006` and `wbc-020` were primary IMU-fault exclusions; the other eight were
unreleased startups, sometimes accompanied by IMU diagnostics.

The reported success rates are therefore **conditional on the requested
exclusions**, rather than overall startup-success rates.

## Motion and timing measurements

Values below are the median across valid trials, followed by the minimum and
maximum across trials.

| Metric | PD | WBC |
|---|---:|---:|
| Net horizontal displacement, m | 2.59 (0.66–4.84) | 13.53 (12.70–14.43) |
| Maximum absolute roll per trial, degrees | 19.45 (12.10–60.13) | 11.86 (10.82–16.11) |
| Maximum absolute pitch per trial, degrees | 40.84 (8.45–69.25) | 5.87 (5.13–7.45) |
| Minimum base height per trial, m | 0.506 (0.234–0.673) | 0.678 (0.674–0.685) |
| Mean real-time factor before the first major fault | 1.0000 (0.9955–1.0057) | 1.0000 (0.9975–1.0026) |
| Fresh C++ source-timestamp matches per trial | 408.5 (86–1500) | 1500 (1500–1501) |
| First major fault after release, failed trials only, s | 5.58 (1.70–9.69) | No major faults |
| Per-trial mean of sampled QP solve time, ms | Not applicable | 0.186 (0.143–0.258) |

Displacement includes trials that subsequently fell or froze, so it must not be
interpreted as successful walking speed. Real-time factor uses post-release
samples before the first major fault. QP timing comes from the 2 Hz
`/wbc/status` stream; it is not a complete distribution of every solver call.

A geometric fall was defined as base height below 0.35 m, or absolute roll/pitch
above 45 degrees. The existing safety supervisor can stop a sustained tilt at
approximately 40 degrees, before the geometric threshold is reached. This
accounts for PD having 20 safety fall trips but 17 geometric falls.

The WBC walking criterion required at least two air/landing cycles per foot:
sole height above 1 cm, swing normal force below 2 N, opposite-foot support above
20 N, at least 40 ms airborne, and subsequent landing force above 5 N. It also
required net horizontal displacement above 0.2 m and completed reference
playback. All 30 valid WBC trials met that criterion.

The default PD launch does not publish the foot ground-truth stream and uses
finite reference playback. Its foot-cycle criterion was marked unevaluable;
this did not cause its stability failures.

## Configuration differences and interpretation

| Setting | PD | WBC |
|---|---|---|
| C++ forwarding / joint feedback | 50 / 50 Hz | 200 / 200 Hz |
| WBC / policy rate | None / 50 Hz | 100 / 50 Hz |
| Reference playback | Approximately 6 s, then `hold_last_frame` | Starts its clock at release and loops |
| Head/target fixtures | Enabled | Disabled |
| Initial foot clearance | 0.02 m | 0 m |
| Enable/release path | Original simulator autostart | Verified C++ enable and forwarding evidence |
| Safety startup / DDS publication | Original startup and publication defaults | 3 s supervisor delay; asynchronous publication |

The default WBC preserves the trained policy impedance loop and applies a
bounded nominal-torque correction of ±0.02 Nm. It is not an independent gait
planner. The comparison supports the observed stability of this complete WBC
configuration; it does not establish how much improvement comes from the QP
rather than timing, playback, or the other configuration differences.

Independent restarts of this deterministic model mainly sample startup and
scheduling variation. The campaign did not test disturbance rejection,
randomized initial conditions, varied terrain, or continuous walking beyond
30 seconds per restart.

## Probe review, shutdown, and supporting checks

The original probe marked `wbc-001` and `wbc-002` as failures because a fresh,
short-lived CLI graph query missed some node names. Neither trial fell or
reported an active control/safety fault. Their sustained fresh C++ command
forwarding, Web HTTP responses, and IMU/joint/odometry streams independently
confirmed the missing components were functioning. Both were classified as
full-chain passes after evidence review; their original JSON results were
preserved. Later trials also used the persistent observer's ROS graph.

All 60 owned launch process groups stopped without timeout or signal escalation
according to the probe. However, **all 60 logs contained shutdown-stage Python
tracebacks after SIGINT**. Runtime outcomes and shutdown diagnostics were
counted separately; a stopped process group is not evidence of traceback-free
teardown. This campaign does not claim clean shutdown behavior.

Supporting simulator, WBC bridge, model-equivalence, and policy component tests
reported **89 passed**. These checks were separate from the 60 ROS integration
trials. The final artifact audit confirmed 74 recorded attempts, 60 valid full
30-second observations, fresh C++ forwarding and Web availability in every
valid trial, and 30 WBC walking-criterion passes.

## Local evidence

Raw logs, agent transcripts, temporary probes, and campaign data remain local
under `test_results/wbc_pd_20261030/`, which is ignored by Git. They are
intentionally excluded from the versioned report:

- `summary.json` and `trials.csv`: aggregate and per-attempt metrics.
- `campaign/progress.json`: execution order, valid counts, exclusions, and paths.
- `campaign/<mode>-<attempt>/trial-*.json` and `.log`: recorded observations and
  actual launch output.
- `graph-discovery-reviews.json`: the two reviewed discovery false negatives.
- `baseline.json` and `final-validation.json`: source snapshot and final checks.

Only this final report is intended for version control; local evidence files
will not be available in a fresh clone unless copied separately.
