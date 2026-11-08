# WBC deprecated for the simulation stack (2026-11-08)

**Decision:** keep the RL policy with the tuned joint-PD path (`pd_profile:=tuned`,
now the `sim.launch.py` default). `enable_wbc` stays available but is deprecated
and not recommended. Test platform: Jetson Orin Nano, ROS 2 Humble, CycloneDDS
on loopback, MuJoCo headless, `scripts/collect_fall_data.py --profile recovery`,
20 s episodes, paired by seed.

## Results

| Condition | Tuned PD | WBC (fixed) | Paired only-PD / only-WBC falls |
|---|---|---|---|
| No push, seeds 900000-900047 | 1/48 | 17/48 | 0 / 16, p < 0.001 |
| Push, seeds 900048+ | 24/46 | 32/49 | 3 / 9, p = 0.15 |

"Fixed WBC" includes `wbc_stale_input_grace_s:=0.15` and
`policy_imu_accel_outlier:=skip`; chain faults count as falls.

## Diagnosis

| No-push variant (48 episodes) | Walking falls | Standing falls |
|---|---|---|
| Tuned PD | 1/24 | 0/24 |
| WBC running, output bypassed (`wbc_bypass:=true`) | 1/24 | 0/24 |
| WBC in path | 6/24 | 4/24 |
| WBC in path, `wbc_gantry_hold:=policy` | 12/24 | 0/24 |

- The WBC node, its extra simulator topics, CPU load and the WBC startup
  sequence do not cause the falls: with WBC bypassed the robot matches PD.
- Standing falls come from zero-gain arming on the gantry: legs hang limp and
  jump to full policy gains at release. Holding the policy command on the
  gantry removes them (simulation-only issue).
- Walking falls remain and cluster 3.2-4.3 s after release (a fixed gait
  phase). WBC adds about 6 ms median / 12 ms p99 policy-to-C++ latency; whether
  latency or the bounded (<= 0.02 Nm) correction causes them was not resolved.
- The shadow fall-predictor node was ruled out as a cause.

## Why not pursue it further

- The RL policy was trained against direct joint PD. A post-hoc WBC changes the
  closed loop; its correction is capped at 0.02 Nm, so it adds latency and
  startup risk without measurable benefit.
- WBC requires simulator ground-truth odometry and foot contact; hardware would
  need state estimation first.
- A WBC that helps would need a redesigned hierarchy (RL commanding task-space
  targets, WBC in the training loop, in-process C++ implementation).

Diagnostic switches kept (default off): `wbc_bypass`, `wbc_gantry_hold`,
`wbc_tick_on_policy`, `wbc_max_correction_nm`, `wbc_stale_input_grace_s`.
