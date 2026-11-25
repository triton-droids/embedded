# Modes and entry conditions

Source: [`core/machine.py`](../humanoid_mode_manager/core/machine.py) (one `_on_<mode>` method per mode),
defaults in `MachineParams` and [`config/mode_manager.yaml`](../config/mode_manager.yaml). Parameter names
are given in brackets.

## Priority on every tick

| # | Condition | Next mode |
|---|---|---|
| 1 | `/safety/estop` true (from any mode) | `estop` |
| 2 | While a controller owns the motors (`homing`, `gantry`, every motion mode), one of: inputs stale (joints > 0.2 s [`joint_timeout_s`], IMU > 0.2 s [`imu_timeout_s`], controller status > 3 s [`status_timeout_s`]); supervisor at ERROR (WARN does not count); controller fault while not falling | `damping` -> `fault` |
| 2 | Same modes, `disable` request | `damping` -> `disabled` |
| 3 | Otherwise the current mode's own rules below | |

**Falling** means any of: tilt > 0.45 rad [`fall_tilt_rad`]; the walking controller's recovery is `rise` or
`straight`; `/safety/fall_warning` with [`use_fall_warning`]. 0.45 rad is below walking_controller's
`max_tilt_rad` (0.5) and the safety trip (40 deg = 0.70 rad), so the mode manager sees the fall first.

## Safety layer

| Mode | Entered when |
|---|---|
| `estop` | `/safety/estop` true |
| `damping` | Priority 2; enable or home failed / timed out; get-up timed out; get-up attempts used up; fallen with `getup: none` |
| `fault` | `damping` for 1 s [`damping_s`] after a fault; `boot` without all inputs for 30 s [`boot_timeout_s`] |

## Lifecycle layer

| Mode | From | Entered when |
|---|---|---|
| `boot` | start | Initial mode |
| `disabled` | `boot` | Joints, IMU and controller status fresh, and an estop message received |
| | `estop` | Estop false again (released by humanoid_safety after its reset) |
| | `fault` | Operator `reset` |
| | `damping` | 1 s after a `disable` request, or after a fall with `getup: none` |
| `enabling` | `disabled` | `enable` with fresh inputs, estop false and the supervisor not at WARN (cpp_control_node's admission) |
| `homing` | `enabling` | ENABLE COMPLETED and [`home_on_enable`]; no COMPLETED within 5 s [`command_timeout_s`] -> `damping` |
| `gantry` | `homing` | HOME COMPLETED; not within 15 s [`home_timeout_s`] -> `damping` |
| | `enabling` | ENABLE COMPLETED without homing |
| | `disabled` | [`external_enable`] (default: same as `dry_run`): first `disabled` after `boot`, gantry released, estop false, inputs fresh. Follows `sim_autostart` |

## Motion layer

| Mode | From | Entered when |
|---|---|---|
| `stand` | `gantry` | `gantry: sim`: `/sim/status` `base_held` false; `gantry: operator`: `release` request |
| | `walk` | No non-zero velocity command for 1 s [`stand_after_s`] |
| | `kick` | Kick done (status idle 1 s) with no velocity command, or kick longer than 5 s [`kick_timeout_s`] |
| | `squat` | After 5 s [`squat_timeout_s`] (placeholder) |
| | `falling` | Upright again: tilt < 0.25 rad [`upright_tilt_rad`] for 1 s [`upright_s`] |
| | `getup` | Tilt < 0.25 rad for 1 s and the walking controller handed back (`recovery` = `walk`) |
| `walk` | `stand` | Fresh (0.5 s [`cmd_vel_timeout_s`]) non-zero (> 1e-3 [`cmd_vel_deadband`]) velocity command |
| | `kick` | Kick done with a velocity command |
| `kick` | `stand`, `walk` | `kick_left` / `kick_right` with `controller: walk` (refused for `policy`) |
| `squat` | `stand` | `squat` request (placeholder: no runtime squat in walking_controller) |
| `falling` | `stand`, `walk`, `kick`, `squat` | Falling (see above) |
| `fallen` | `falling` | Angular speed < 0.3 rad/s [`rest_angular_speed`] for 0.5 s [`rest_s`] |
| | `getup` | `getup: policy` only: at rest and not upright for 2 s, 2 s after entering `getup` (the policy gave up) |
| `getup` | `falling` | `getup: walking_controller` and its recovery is `rise` / `straight` (skips `fallen`: it is already getting up) |
| | `fallen` | Attempts < 3 [`max_getup_attempts`]; otherwise `damping` -> `fault` |

Get-up attempts reset after 20 s [`getup_reset_s`] in `stand` / `walk` after a get-up; a fall before that
keeps counting. `getup` gives up after 30 s [`getup_timeout_s`].

## Navigation sub-mode (published only)

| Sub-mode | Condition on `/localization/status` |
|---|---|
| `navigate` | `initialized` and `vo_age_s` <= 0.5 s [`vo_lost_s`] |
| `local_only` | Otherwise |
| `hold` | `local_only` for 3 s [`hold_after_s`] |
| `unknown` | No status received |
