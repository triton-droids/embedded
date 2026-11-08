# Offline Safety evaluation — 2026-11-07

Model: tasksource/tasksource-decider-nano, unmodified pretrained weights.
Revision: `2c7c01ffe5aba74dd15968cf38b0fdc83d1f3810`.

48 synthetic IMU scenarios, each with both option orders (96 requests).
Reference labels come from the existing SafetySupervisor and its fall,
freshness and frozen-IMU checks. English rules and compressed sensor histories
are provided to the model. This is check-layer rule agreement, not real-world
safety accuracy. Motor, joint, CAN, battery and collision checks are not tested.

| Metric | Result |
|---|---:|
| Agreement | 58/96 = 60.42% |
| Required-stop recall | 58/58 = 100% |
| Dangerous missed stops | 0/58 |
| False stops on continue labels | 38/38 = 100% |
| Balanced accuracy | 50% |
| Always-stop baseline | 60.42% |
| Option-order flips | 0/48 |
| CPU median latency | 1023 ms |
| CPU p95 latency | 1440 ms |

The model selected stop on every request. Thus its agreement equals the
always-stop baseline. Zero missed stops here is explained by always stopping,
and provides no evidence that the model discriminates hazardous states.
Normal posture, pending falls, ignored glitches and startup grace were all
misclassified. The boundary group scored 2/8, with only the over-limit scenario
correct. Other groups requiring a stop scored 8/8 each.

CPU PyTorch 2.6.0+cpu, transformers 5.17.0, four threads. Timing includes the
Python answer call, after model loading; first inference is included. This is
not a GPU or optimized deployment benchmark. Existing Safety evaluation is
configured for 100 Hz (10 ms periods).

This zero-shot configuration does not support replacing the deterministic
supervisor. Results apply only to this fixed prompt, dataset, model revision
and runtime. No fine-tuning was performed. Follow-up evaluation should use
independent recorded/simulated episodes, broader fault coverage and an
explicit assessment of any redesigned input representation.

See `README.md` for reproduction and `results.json` for every input,
prediction, score, timing and model/source hashes. Softmax scores have not
been calibrated as safety probabilities.
