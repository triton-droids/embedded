# Motor configuration alignment

Aligned to `robstride_data` commit `2728fe19eb17908ac5bd4d88c8e9a131111a9afd`.
The canonical source is `ctrl_scripts/run_policy_config.json`, rather than that
branch's gain tuner (which disagrees on directions for IDs 2, 3 and 8).

- CAN IDs 1–10 and mixed rs-04/rs-03/rs-02 models match the source policy config.
- Directions, joint limits, per-joint gains, default pose, action scale, action
  clipping, and soft limits match the source policy config.
- Both the legacy bridge config and tracking deployment override remain 50 Hz.
- ONNX joint order and original metadata remain intact for export validation;
  deployment values are applied by joint name after validation. The override
  intentionally changes observation centering and action decoding from the
  original tracking training settings. This is not a validation of policy quality.
- The CAN conversion now supports the source four-bar ankle mapping, with
  continuous solver guesses and position-derived ankle velocity.
- The synchronization script preserves explicit deployment settings and gains.

`hardware_verified` remains false. IDs/signs are copied configuration, not live
hardware verification. Encoder offsets are zero placeholders. The source runner
also derives startup multi-turn offsets from live readings; that startup
calibration has not been added to the ROS driver. Verify physical zero offsets
before enabling hardware. No CAN commands were issued during this change.

Validation: compared all ten mappings/models/signs/gains/limits against source;
checked both ankle conversion round trips at -0.5, 0 and 0.4 rad; checked runtime
name mapping, 50 Hz and hardware guard; Python compilation and whitespace checks
passed. Full ROS/ONNX tests were unavailable in the active Python environment
because python-can and ONNX dependencies are missing.
