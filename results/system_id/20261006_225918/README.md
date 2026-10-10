# System-ID fit, session 20261006_225918

Actuator parameters for the 10 leg joints, fitted to one-joint trials on the gantry.

- **Data:** 60 trials (Oct 6, plus 8 re-runs on Oct 7) recorded with `ctrl_scripts/sysid_logger.py`. The raw `.npz` files are not in git: they live in `logs/system_id/20261006_225918/` on the robot laptop, and in the GPU handoff package.
- **Excluded:** 8 trials, each listed with its reason in `fit_exclude.json`. These are motor 5 trials taken while the physical right ankle was binding at about 1 Nm. **The ankle needs a mechanical check.**
- **Fit:**
  ```bash
  ~/Documents/simulation/.venv-sysid/bin/python utils/sysid_fit.py logs/system_id/20261006_225918 --base yaw --out fit_v5
  ```
  It needs a CPU venv with mujoco, numpy and scipy, and the simulation repo's `codex/hardware-tracking-retrain` model.
- **Model:**
  - kp is kept at the commanded value. Per joint, the fit gives armature, Coulomb friction, a kd scale and a command delay.
  - The base yaw is replayed from the IMU, because the robot yaws on the gantry strap.
- **Quality:** every joint beats the training defaults on holdout trials. Knees 0.16–0.25°, thighs 0.12–0.16°, ankles 0.20–0.21°, hips 0.14–0.20°.
- **Caveat:** the hip values are gantry-confounded and flagged `untrusted`, so training uses widened ranges for them.

**Used by:** `train_velocity.py --fit sysid_params.json`, in the simulation repo.
