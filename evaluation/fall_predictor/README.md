# Small pre-estop fall predictor

Run from `embedded/`, using an environment with NumPy and CPU PyTorch:

```bash
python evaluation/fall_predictor/train.py \
  --source data/fall_collection/overnight_20261107 \
  --output data/fall_training/my_run
```

Output must be a new directory. The script uses 4 CPU threads and defaults to
50 epochs with early stopping after 5 epochs without improvement in validation
average precision. It does not start ROS, change Safety or send motor commands.

- Excludes manifest-invalid episodes, unknown labels, gantry/faulted histories,
  and any 20-frame history containing active estop. This targets warning before
  existing Safety trips rather than recognition of an already disabled robot.
- Rechecks finite features, time order, history continuity and future labels.
- Stratifies episodes by reference mode, fall outcome and actual push, then splits
  approximately 70/15/15. All windows from one episode stay in one split.
- Fits feature normalization using training endpoints only.
- Balances each epoch with all training positive windows plus an equally sized
  random subset of negative windows. Validation/test are never balanced.
- Uses an MLP 160→64→32→1 (12,417 parameters), ReLU, Adam, lr 0.001, batch 128,
  binary cross-entropy. Normalization is embedded in the model.
- Chooses the highest threshold with >=95% validation positive-window recall.
  Test scores are generated only after checkpoint and threshold selection.

Artifacts: `config.json`, episode `splits.json`, `history.json`, model weights,
`model_torchscript.pt`, `metrics.json`, `test_predictions.npz`, and `REPORT.md`.
The TorchScript model accepts float32 `[batch,20,8]` in the feature order from
config, produces logits, and includes train-only normalization. Apply sigmoid
and compare with the saved threshold. Scores are not calibrated probabilities;
training undersampling changes their class prior.

Metrics include natural-prevalence precision/recall, confusion matrix, AP/AUC,
per-event detection within 300 ms before contact and before active estop,
warning lead relative to existing Safety, false alarm onsets with 1-second
cooldown per negative-exposure hour, and CPU forward timing. The existing
Safety estop state itself is always false in these evaluation windows, so a
simple current-tilt >40-degree baseline is also reported.

This is an initial same-simulator, same-controller baseline. No-push trials and
motion prefixes can be very similar across different episodes. Holding out
entire episodes prevents shared-window leakage but does not establish new
scenario, new robot dynamics or real-world generalization. Events without a
usable positive window before estop are counted separately. Future outcomes
include existing Safety's motor disable. No control deployment is performed.

Checks:

```bash
python evaluation/fall_predictor/test_train.py
```

## Unattended search

```bash
python evaluation/fall_predictor/autotrain.py \
  --source data/fall_collection/overnight_20261107 \
  --fixed-splits data/fall_training/20261107_v1/splits.json \
  --output data/fall_training/auto_run --hours 6.5
```

Use an environment with CPU PyTorch, for example the environment used for the
first run. This does not require ROS or networking. Background launching should
use an independent session (`setsid`) and redirect stdin/stdout/stderr.

The runner loads data once, retains the fixed original train/validation folds,
trains small/base/wide MLPs and a small temporal CNN, and searches learning
rate, negative:positive sampling ratio (1, 2 or 4), dropout and weight decay.
It also selects validation-only threshold and consecutive-frame confirmation
(1, 3 or 5). No test predictions or robot commands are produced.

Default stopping rules: 6.5 hours maximum including loading; at most 96 trials;
24 completed trials without improvement; 3 consecutive errors; or less than
2 GiB free disk. Individual candidates early-stop on validation AP. The best
validation candidate must meet 95% eligible-event recall and 50 ms median lead
before existing Safety to be marked qualified; among those, false alarm onsets
per negative-exposure hour are minimized. If none qualifies, output explicitly
says so. These are search gates, not deployment acceptance criteria.

`status.json` records PID/progress, `best.json` points to an immutable trial
model and its alarm policy, `trials.jsonl` lists candidate results, and each trial
has configuration/history/checkpoints. Source and split snapshots are saved.
SIGTERM to the status PID gracefully ends the search, retaining completed best
results. Stop latency includes the current bounded validation/policy operation.
A failed trial is logged and skipped. Choose a new output directory for each run.

Validation is repeatedly optimized, so final performance needs newly collected
independent scenarios. The old test fold remains reserved and is not reevaluated.
Test the search helpers with `python evaluation/fall_predictor/test_autotrain.py`.

## Recovery collection and dual horizons (v2)

`run_v2_pipeline.py` chains three bounded stages: new recovery-profile simulation
collection, a nested 300/600 ms CNN, and one evaluation of the selected candidate
on newly collected sealed test families. Start using a CPU PyTorch interpreter:

```bash
python evaluation/fall_predictor/run_v2_pipeline.py \
  --output data/fall_training/dual_v2 --episodes 384 --hours 6.5
```

The root `status.json` identifies the active stage and child PID. Collection
progress is in `collection/status.json`; subsequent training progress is in
`training/status.json`. Source/config snapshots and stage logs are retained.
The collector uses domain 88, localhost only, no camera/viewer, and active Safety.
A 2.5-hour collection budget and the shared 6.5-hour pipeline budget include
startup overhead. A failed stage does not silently advance. SIGTERM to the root
PID stops its child; completed episode/checkpoint files remain.

Recovery scenarios use 48 families: reference mode x 3 floor-friction settings
(0.65, 0.85, 1.15) x 8 force-direction sectors. Repeats vary force magnitude,
duration and delay; some cycles have no force. The controller and motor/Safety
interfaces remain the existing stack. A recovery is recorded only after an
actual push, no contact-defined fall, no post-push estop, at least 3 seconds of
post-push observation, and a final second with absolute roll/pitch below 25 deg.
This is a collection heuristic, not proof of indefinite stability.

The family-to-fold map is fixed and saved before collecting. Entire families
stay in one fold (32 train / 8 validation / 8 test). Three pilot families are
training-only. Old v1 test episodes are excluded entirely; old train/validation
folds remain development data. Final new test predictions are made once, after
model and both temporal alarm policies are fixed.

Both heads are trained from original raw trajectory outcomes, with 1 ns numerical
tolerance. Histories containing estop, holds, faults or invalid sample gaps are
excluded. The 600 ms future must be observed unless a confirmed fall occurs
within it. The output construction guarantees sigmoid(300 ms) <= sigmoid(600 ms).
The shared CNN outputs two logits; normalization is embedded. Sampling balances
imminent falls (<=300 ms), later falls (300–600 ms), and safe windows, with targeted
new recovery-negative sampling. Minimum recovery-episode gates are 8 train,
2 validation and 2 test. Insufficient recovery data stops training explicitly.

Eight training configurations are compared using validation only; checkpoint
selection uses mean AP across heads, and candidate selection minimizes 600 ms
false alarm onsets under the existing validation recall/lead gates. Both head
policies are saved. The one-shot test reports both horizons, actual recovery
alarms, and events without eligible input separately. These remain simulation
results under the current controller including Safety's disable response;
the new model is not installed in the control loop.

Tests: `python evaluation/fall_predictor/test_dual.py` includes a complete small
synthetic fixture run; this verifies the data/export/evaluation flow, not physical
prediction performance. Collection boundary and recovery tests run in the ROS
workspace with `pytest simulation/humanoid_sim/test/test_collection.py`.
