# Unattended fall-model search

Status: status.json. Best validation candidate: best.json (immutable trial model
path plus threshold and required consecutive frames). Logs: trials.jsonl and
trial_*/history.json. No test evaluation and no deployment are performed.

Three CPU threads; bounded wall budget, trial count, validation stagnation,
free-disk and consecutive-error stops. Per-trial checkpoints preserve progress.
Three model families plus a small CNN, learning rates, class sampling ratios,
dropout and weight decay are searched. Fixed v1 train/validation episode folds
are reused. All candidates use only pre-estop labelled histories. Validation
selection targets >=95% eligible-event recall and >=50 ms median lead over
existing Safety, then minimizes false alarm onsets per negative-exposure hour.
If no candidate meets the gates, best.json is marked qualifies=false; the
runner still never deploys a model.

An alarm requires confirm_frames consecutive scores above threshold, with
resets on episode changes or sample gaps over 20 ms. A 1-second cooldown is
used for counting false alarm onsets. Normalization is in the TorchScript
model; outputs are logits. Apply sigmoid and the saved temporal policy.
Scores are uncalibrated. The old test fold is reserved without reevaluation;
since earlier baseline test results are known and validation is repeatedly
searched, final performance requires newly collected independent scenarios.
Same-simulator and repeated-trajectory limitations still apply. Validation
numbers must not be presented as fresh test performance.
