# Deprecated

Kept for reference and still built by colcon, but not developed further.

| Path | What it is |
|---|---|
| [wbc_control_node](wbc_control_node/README.md) | RL policy → TSID bridge. Still started by `sim.launch.py enable_wbc:=true`; the model-based walking controller ([control/walking_controller](../control/walking_controller/README.md)) replaces it. |
| [wbc_prototype](wbc_prototype/README.md) | Isolated (no ROS) WBC model-equivalence and stance tests, and `sync_wbc_model.py`, which regenerates the leg URDF dynamics from the MuJoCo model. Moved here from `experimental/`. |
