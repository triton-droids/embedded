# Running the velocity policy

`ctrl_scripts/run_velocity_policy.py` runs a policy trained by `cad/train_velocity.py` (simulation repo) at 50 Hz. It's driven from a gamepad and uses the same CAN path, torque guard and fault checks as `sysid_logger.py`.

## Before every session
- **Battery charged.** Pass `--vbus-min` for its chemistry: 35 for 10S Li-ion, 36 for 12S LiFePO4.
- **CAN up:** `ip -d link show can0` shows ERROR-ACTIVE, and `qlen 1000` (`sudo ip link set can0 txqueuelen 1000` after every `slcand` restart).
- **Zeros:** legs hanging straight, `utils/validation_code/joint_direction_monitor.py --seconds 5` shows every motor within ±1°. Don't run `zero_out.sh` unless every joint is held straight.
- **IMU plugged in.** The policy refuses to start without it.
- **Robot on the gantry**, with someone holding the E-stop.
- **Never paste terminal output back into a terminal.** It runs the lines as commands. That's what erased six log files on Oct 6.

## Check the policy, its contract and the gamepad (no CAN, nothing moves)
```bash
./.venv/bin/python ctrl_scripts/run_velocity_policy.py --policy <run>/<run>.onnx --check
```
- The contract is read from `velocity_contract.json` next to the ONNX, the file the policy was trained with.
- The runner refuses to start if the ONNX metadata (joint order, stand pose, action scale, observation names) disagrees with the contract.
- The check prints live stick and button numbers. If your pad differs from the defaults, fix them in `ctrl_scripts/run_velocity_config.json`. Defaults (Xbox layout):

| Control | Default |
|---|---|
| Deadman | LB (button 4) |
| Home to stand pose | A (button 0) |
| Start the policy | Start (button 7) |
| Stop | B (button 1) |
| Forward | left stick Y |
| Yaw | right stick X |
| Lateral | off |

## Run
```bash
./.venv/bin/python ctrl_scripts/run_velocity_policy.py --policy <run>/<run>.onnx --yes --vbus-min 35
```
1. **Enter:** all motors enable and hold the pose they're in.
2. **Hold the deadman and press A:** the legs move slowly to the stand pose (at least 4 s; at most 0.5 rad/s).
3. **Lower the gantry** until the feet carry the weight, with slack in the strap.
4. **Hold the deadman and press Start:** the policy runs. Push the left stick forward to walk.
   - Commands ramp at 0.5 m/s² and are limited to the contract's range: forward −0.1 to 0.4 m/s, yaw ±0.3 rad/s.
5. **Release the deadman:** the command ramps to zero and the policy stands in place.
6. **Press B:** stop. The motors hold briefly, then disable.

## What stops the run by itself
- The torque guard: 108/54/15 Nm sustained, RMS above the rated torque, or above 85 °C.
- 5 missed replies in a row.
- A status fault bit, a motor leaving run mode, or a type-21 fault frame.
- VBUS below `--vbus-min`, or sagging more than 3 V under load.
- Tilt over 40°.
- A non-finite action.
- The IMU reader dying.
- The gamepad silent for 1 s.
- `--max-seconds` (default 300).

## Logs
Each run writes `logs/velocity/<timestamp>/velocity_<stamp>.npz` (read-only) and `manifest.json`. They record every step:
- observation, action, the four target stages, and what was sent;
- mode, command, gamepad state;
- reply timestamps, fault and mode bits, VBUS, raw CAN frames.

Summarise them with `utils/sysid_report.py logs/velocity/<timestamp>`.
