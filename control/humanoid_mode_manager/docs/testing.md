# Testing

## Unit tests

```bash
python3 -m pytest control/humanoid_mode_manager/test
```

`test/test_machine.py` (24 tests) drives `ModeMachine` through time without ROS: lifecycle, estop, stale inputs,
supervisor WARN / ERROR, walk / stand, both get-up paths, the attempt limit, kick, external enable and the
navigation sub-mode.

## Simulation dry runs (2026-11-22, laptop, Jazzy)

All with `mode_manager:=true` (dry run: the node only follows; `sim_autostart` enables the motors).

| Run | Setup | Mode sequence | Notes |
|---|---|---|---|
| Legs | `controller:=model_walk`, 0.08 m/s for 16 s, 150 N push | boot, disabled, gantry, stand, walk, stand, damping, estop | 43 steps. The push makes walking_controller latch a tilt fault and disable the motors; the simulator pauses physics, inputs go stale (`damping`), then the safety IMU timeout (`estop`). No get-up on the legs model. |
| Full body, get-up | `robot:=full_body_arms walk_fall_recovery:=true`, RTF 0.03, push after 4 steps | stand, falling, getup, stand | `falling` 10 ms after the controller's own tilt fault (0.548 rad seen), `getup` 0.2 s later, hand-over after the 6 s straightening timeout, `stand` 1 s later. |
| Full scenario 1 | `tools/run_scenario.sh` | stand, walk, falling, getup, damping, fault | Get-up went rise -> straight -> rise three times; `getup` timed out after 30 s. |
| Full scenario 2 | `tools/run_scenario.sh` | stand, walk, falling, getup, stand, walk, stand | Complete: get-up 15.1 s (simulated), walked again 4 s, stopped. |

Videos (1x simulated time, state panel and mode timeline): `videos/mode_manager_scenario_full.mp4` (scenario 2),
`videos/mode_manager_scenario_getup_timeout.mp4` (scenario 1).

Findings fixed on the way: the node did not follow an external enable (`external_enable`); a supervisor WARN
during start-up was treated as a fault (now only ERROR, as in cpp_control_node); `fall_tilt_rad` 0.6 was above
the walking controller's 0.5 rad (now 0.45).

Start-up under CPU load is fragile independently of the mode manager: with another simulation running, the
safety IMU freshness check (0.3 s) tripped at start-up in 3 of 5 runs and `sim_autostart` timed out once.

## Scenario video

```bash
control/humanoid_mode_manager/tools/run_scenario.sh /tmp/mm_scenario        # RTF 0.03, 8 threads
control/humanoid_mode_manager/tools/run_scenario.sh /tmp/mm_scenario 0.08 16 # 24-thread host
```

It starts the full-body simulation, runs `tools/record_scenario.py` (start -> ready 2 s -> walk 4 s -> 150 N push
-> get-up -> settle 2 s -> walk again 4 s -> stop 4 s, all simulated time; stops early on `fault` / `estop`) and
renders with `tools/render_scenario.py`. The get-up MPC needs about 20-30 min wall time at RTF 0.03.
