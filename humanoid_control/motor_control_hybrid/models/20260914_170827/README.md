# Deployment tracking model

Recovered from the original simulation export, without retraining or changes.

- Model: `20260914_170827.onnx`
- SHA256: `90fc691abbf43b89a55956493da6ab19c59105f9167f29f8f93fc317b68c9309`
- CPU ONNX Runtime; 56 observation inputs, 10 joint actions, 50 Hz.
- Reference motion is embedded in ONNX (299 frames, then hold last frame).
- XML snapshots are retained for `scripts/sync_tracking_motor_config.py`;
  inference does not require MuJoCo, CUDA, training dependencies or simulation.
- ROS installs this directory into the motor_control_hybrid package share.
- The launch and node use the installed model by default; `model_path` can override it.

The existing motor registry matches this model. No registry resync is needed.
