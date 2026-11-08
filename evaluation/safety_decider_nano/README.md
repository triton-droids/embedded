# Safety decider nano: offline rule agreement

Model: `tasksource/tasksource-decider-nano`, revision
`2c7c01ffe5aba74dd15968cf38b0fdc83d1f3810`.

This benchmark compares unmodified pretrained model choices with the existing
`humanoid_safety.checks` IMU supervisor. It does not operate hardware or change
runtime safety code. It is a synthetic smoke benchmark, not a measurement of
real-world fall detection or the effectiveness of protective actions.

48 deterministic scenarios cover normal posture, confirmed and pending falls,
gyro-detected glitches, stale data including source delay, frozen data, invalid
orientation, latch retention after recovery, startup grace and tilt boundaries.
The reference uses fall detection (40 degrees, 3 confirmations), IMU freshness
(0.30 seconds, 3-second startup grace) and stuck detection (50 identical samples).
Hardware checks are disabled, matching the checked-in safety configuration.
The test targets check-layer observations; ROS message rejection and reset
requests are outside its scope. Inputs are English, with numerical histories
compressed losslessly into runs of identical sensor values. Rules are supplied
in the question. Expected labels and internal check results are not given to
the model. No training or prompt selection based on outcomes is performed.

Each scenario is evaluated twice, reversing option order. The resulting 96
requests are correlated; they are not 96 independent real-world trials.
`cases.json` preserves the reference inputs and labels. `results.json` contains
predictions, softmax scores, timing, metrics, and inference-file hashes after a
successful run. A partial run is explicitly marked `partial`.

Download the fixed model revision, including the upstream `decider.py`, into
a local directory. Use a compatible isolated PyTorch/transformers installation
(the model config specifies transformers 5.17.0). Exact versions are in
`requirements.txt`. This run used CPU PyTorch 2.6.0+cpu with four threads.
The host's incompatible optional torchvision/torchaudio/librosa packages were
hidden from dependency discovery by a temporary launcher; model code, weights,
tokenization and scoring were unmodified. A clean environment avoids that
workaround.

```bash
python embedded/evaluation/safety_decider_nano/evaluate.py \
  --model-dir /tmp/safety-nano-model \
  --output embedded/evaluation/safety_decider_nano/results.json
```

Generate reference cases without model dependencies:

```bash
python embedded/evaluation/safety_decider_nano/evaluate.py \
  --model-dir /tmp/unused --generate-only \
  --output embedded/evaluation/safety_decider_nano/cases.json
```

Report overall agreement, dangerous false negatives (required stop predicted
continue), unnecessary stops, order sensitivity, and latency. Softmax scores
are not validated safety probabilities. A strong result here alone cannot
establish suitability for replacing a deterministic safety supervisor.
