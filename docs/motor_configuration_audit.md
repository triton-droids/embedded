# Motor configuration cross-check — 2026-10-07

Update following this audit: `config/motors.yaml` now contains the synchronized
ten-joint simulation registry; the original two-motor configuration is preserved
as `config/bench_motors.yaml`. The CAN driver now validates mappings before bus
access, applies direction/encoder offset and target position/velocity/optional
torque limits, uses per-joint gain defaults, and excludes stale feedback. The
findings below describe the pre-update state. See
[current synchronization behavior](tracking_policy_ros2.md#synchronized-motor-registry).

Compared this embedded checkout with the sibling simulation checkout, especially
the deployed export `logs/legs_tracking/20260914_170827/20260914_170827.onnx`
and its saved training XML. This is a file/configuration audit, not a physical
CAN discovery or motor calibration. No motor settings were changed.

## Active tracking order and PD gains

The ONNX metadata joint order matches the saved `tracking_body_order.json`, XML
actuator order, embedded tracking fake-joint list, and RViz URDF movable-joint
order. ONNX stiffness/damping exactly match XML `kp`/`kv`.

| Action index (zero-based) | Joint | Tracking Kp | Tracking Kd | Older Torch bridge Kp | Older Torch bridge Kd |
| --- | --- | ---: | ---: | ---: | ---: |
| 0 | left_hip1_joint | 100 | 1 | 300 | 20 |
| 1 | left_hip2_joint | 100 | 1 | 200 | 5 |
| 2 | left_thigh_joint | 100 | 1 | 100 | 2 |
| 3 | left_knee_joint | 80 | 1 | 100 | 20 |
| 4 | left_ankle_joint | 20 | 1 | 35 | 0.8 |
| 5 | right_hip1_joint | 100 | 1 | 200 | 5 |
| 6 | right_hip2_joint | 100 | 1 | 300 | 15 |
| 7 | right_thigh_joint | 100 | 1 | 100 | 2 |
| 8 | right_knee_joint | 80 | 1 | 100 | 20 |
| 9 | right_ankle_joint | 20 | 1 | 45 | 1 |

These are PD gains; the tracking message has no Ki/integral term. The simulation
XML explicitly labels these gains and its +/-120 Nm actuator force limits as
unverified hardware specifications. The tracking node loads gains directly from
ONNX metadata and sends them in `/policy/desired_motor_subset`. The C++ relay
preserves supplied gains; its 40/1.5 values are fallbacks. The fake motor stores
gains but moves through a rate-limited target-following model, not a PD torque
simulation, so fake movement cannot validate these gains.

## Separate legacy policy contract

`config/policy_bridge_config.json` belongs to the older Torch bridge. Its policy
order interleaves sides: L hip1, R hip1, L hip2, R hip2, L thigh, R thigh, L knee,
R knee, L ankle, R ankle. Its output/real order is left five then right five, and
the bridge explicitly maps by name. This is a different model contract, not
proof that the active ONNX node has an order bug.

Additional differences: legacy rate 60 Hz versus tracking 50 Hz; legacy action
scale 1.0 (thighs 0.3) versus tracking 0.2 for every joint; legacy nonzero default
hip1/knee/ankle positions versus tracking zero default positions; legacy action
clipping versus tracking raw actions. Do not substitute that JSON for ONNX
metadata. Saved simulation joint limits, embedded RViz URDF limits, and legacy
bridge limits agree numerically for all ten joints; agreement does not establish
physical calibration or enforcement in CAN control.

## Physical motor IDs and driver gaps

- `config/motors.yaml`: only test_joint -> CAN ID 12 and test_joint2 -> CAN ID 13,
  both `rs-03` on can0. No policy leg joint is configured.
- `config/control_config.yaml`: four arm joints -> CAN IDs 21, 22, 23, 24. No
  policy leg joint is configured. This file also warns its models need correction.
- Neither the active simulation XML nor ONNX metadata defines physical CAN IDs.
  No leg CAN-ID mapping was found in the inspected simulation configuration/code.
  An action index is not a physical motor ID.
- `motors.yaml` has 11 inversion entries despite defining only two motors and
  the policy using ten joints. The current CAN node does not read this vector.
- `python_can_node.py` reads joint name, motor_id, model and can_interface for
  each motor, but does not apply configured direction, inversion, offsets,
  min_position/max_position, per-motor kp/kd, or actuator_type. It reads the global
  default_master_id but does not pass it explicitly to RobstrideBus; per-motor
  master_id is also unused.
- Command-supplied kp/kd override global YAML KP=10/KD=0.2. Adding gains to a
  per-motor YAML entry alone will not make this driver use them.
- Policy feedback is reordered by joint name. CAN status indices follow YAML
  motor insertion order, while published JointState order follows feedback
  buffer insertion order. Consumers must use joint names, not array offsets.
- CAN feedback publication reuses cached samples with a new host timestamp
  without checking each cached motor's last receipt time. A ROS JointState rate
  alone therefore cannot establish fresh physical feedback.
- Position/motion commands clamp desired velocity but send target position and
  torque directly. XML/URDF position and torque limits are not transferred into
  this CAN driver by the current tracking launch.

## Result

The active simulation-to-embedded tracking order and PD metadata agree. Real
deployment is incomplete: supply the verified ten-joint CAN mapping and implement
calibrated signs/offsets, per-motor feedback freshness, physical command limits,
and validated hardware gains/stop behavior before connecting policy output.

Primary files:

- `humanoid_control/motor_control_hybrid/motor_control_hybrid/tracking_onnx.py`
- `humanoid_control/motor_control_hybrid/motor_control_hybrid/tracking_policy_node.py`
- `humanoid_control/motor_control_hybrid/motor_control_hybrid/python_can_node.py`
- `humanoid_control/motor_control_hybrid/config/policy_bridge_config.json`
- `humanoid_control/motor_control_hybrid/config/motors.yaml`
- `humanoid_control/motor_control_hybrid/config/control_config.yaml`
- `simulation/logs/legs_tracking/20260914_170827/chrobot_16kg_actuated.xml`
- `simulation/logs/legs_tracking/20260914_170827/chrobot_16kg_candidate.xml`
- `simulation/logs/legs_tracking/20260914_170827/tracking_body_order.json`
