# Motor safety limits

What bounds a RobStride joint between a ROS command and the CAN frame, and
where each number comes from. Nothing here has been tested on the robot yet.

## Driver (python_can_node and robstride_can_node)

Both CAN backends apply the same rules (`motor_configuration.MotorSettings`
and its C++ port in `robstride_codec.hpp` / `robstride_registry.hpp`):

| Guard | Rule | Configured by |
|---|---|---|
| Position | target clipped to `[min_position, max_position]` | motors.yaml (trained limits) |
| Velocity | clipped to `MAX_VEL_RAD_S` | motors.yaml |
| Feedforward torque | clipped to `max_torque_nm` | motors.yaml (model peak) |
| Gains | kp, kd reduced to `max_kp` / `max_kd` (default: the joint's trained kp, kd) | motors.yaml, optional |
| Tracking error | without fresh raw feedback a position/motion frame becomes a zero-gain, zero-torque probe (RobStride answers only commands, so refusing to send would never refresh the feedback); otherwise its target stays within `MAX_POSITION_ERROR_RAD` (2.0) of the measured motor position. This is a coarse guard only: the policy holds 0.3-1.2 rad errors on purpose (ankles), and in simulation a 0.5 rad clamp or a torque-budget clamp made the robot fall | motors.yaml |
| Feedback jump | consecutive raw samples (≤ 0.1 s apart) may not differ by more than `1.5 * no-load speed * dt + 0.05 rad`; otherwise the transport fault latches and every motor is disabled | model no-load speed, or `max_motor_speed_rad_s` |
| Motor CAN watchdog | `canTimeout` (0x7028, 20000 = 1 s) is written at every ENABLE while the motor is disabled, read back with a type-17 request, and the ENABLE frame is sent only when the value matches; a missing reply is re-requested every 50 ms, a wrong one never enables. A motor that hears nothing for that long resets itself (host crash, cable cut) | `FIRMWARE_CAN_TIMEOUT_S: 1.0`, per motor `firmware_can_timeout_s`; must exceed `command_timeout_s` (0.5 s) |
| Unexpected reset | an MIT joint whose fresh status leaves Motor mode without a host DISABLE (the motor watchdog fired, undervoltage, fault) latches the transport fault and disables every motor | always on |

The feedback-jump guard catches a mechanical-zero write sent on the bus while
the stack runs (the coordinate system shifts by the old position in one
sample), an encoder fault, or another motor answering with the same ID.

In a MuJoCo closed loop of the tracking policy through `MotorSettings.command()`
the registry values leave the trajectory identical to the unguarded baseline
(0 % of frames clamped); `MAX_POSITION_ERROR_RAD: 0.5` clamps 54 % of frames and
the robot falls.

## Models and limits from the CAD

`model` was checked against the visual STLs in `humanoid_leg_description`:
the hip1 and knee housings measure about 120 mm (RS04, 120 x 120 mm) and hip2
and thigh 82-94 mm (RS03). The ankle RS02 drives the foot through a four-bar
linkage and could not be located in the meshes.

Each joint was swept through the meshes, one at a time with the others at
zero, to the first new contact:

| Joint | Trained | CAD first contact | Contact |
|---|---|---|---|
| hip1 | ±90° | -114° / +112° | upper leg vs torso |
| hip2 (left) | -90° / +25° | -158° / +25.5° | the two legs touch at +25.5° |
| thigh | ±45° | none | free |
| knee | -120° / 0° | -132° / +133° | **no stop at 0°: hyperextension is only blocked in software** |
| ankle | ±34° | none (1 mm graze only) | linkage rods are not in the URDF |

These go into motors.yaml as `hard_min_position` / `hard_max_position`, and
both backends refuse a registry whose trained range leaves them. Cables,
linkage rods and two-joint poses (both legs adducted, for example) are not in
the sweep, so the hard values are upper bounds, not operating limits.

## Supervisor (humanoid_safety)

`safety.yaml` now enables the motor temperature, fault-bit, torque, feedback,
joint position, joint velocity and CAN-link checks. The CAN nodes publish
per-motor `fault_bits` in `/motor_telemetry`. hybrid_control.launch.py passes
its own `motor_config_file` to the safety node, so both read one registry.

## Tracking policy start

`tracking_policy_node` (in `joint_states` feedback mode) refuses to start when
the first target is more than `start_pose_tolerance_rad` (0.5) from the
measured pose; in simulation the first target is 0.25 rad away (an ankle).
There is no kp ramp: in simulation any ramp changed the policy's start and
made the robot fall when released from the gantry.

## Not covered by software

- A physical emergency stop that cuts motor power without CAN.
- Raw CAN frames sent with `cansend` while the stack runs: the jump guard
  detects a re-zero after the fact; it cannot stop the frame.
- `limit_torque` (0x700B) and `limit_cur` (0x7018) are not written. The RobStride
  manual documents `limit_cur` for velocity/CSP modes only, does not say that
  `limit_torque` bounds the operation-control (MIT) torque, and asks users not
  to change the torque limit; MIT torque is bounded by the feedforward clip, the gain ceilings and the
  tracking-error clamp above.
- The motor watchdog has been exercised against simulated motors only: the
  python-can virtual bus unit tests and `test/can_backend_diff.py` on vcan
  (Jetson, both backends: identical lifecycle/parameter frames on all ten
  motors, canTimeout written and read back before every ENABLE). The diff test
  widens `MAX_POSITION_ERROR_RAD` because the clamp makes motion frames follow
  feedback timing. Still to do on one real motor:
  `scripts/robstride_watchdog_commission.py --physical` (zero gain throughout)
  checks the readback and that the motor resets after 1.5 x the timeout of
  silence.
