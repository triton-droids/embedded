# First simulated pre-estop fall predictor

12,417-parameter MLP, 20 x 8 IMU history, 300 ms prediction horizon.
Only pre-estop histories are used. Valid episodes are split 70/15/15 within
mode/outcome/push strata before windows are used. Each training epoch contains
all training positives and an equal random subset of negatives. Validation and
test retain natural prevalence. Checkpoint selection uses validation average
precision; threshold uses validation >=95% window recall. Test is evaluated
once after selection. No integration into Safety or robot control.

Best epoch: 2; threshold: 0.734506.

| Test metric | Value |
|---|---:|
| Window recall | 97.17% |
| Window precision | 51.76% |
| False positive rate | 1.26% |
| Average precision | 0.8594 |
| ROC AUC | 0.9965 |
| Missed positive windows | 56 / 1978 |
| Eligible fall episodes detected | 130 / 131 |
| Median warning lead before contact | 294.9999999999937 ms |
| Median lead before existing Safety estop | 149.99999999999682 ms |
| False alarm onsets per negative-exposure hour | 389.76240944066035 |
| CPU forward median / p95 | 0.762 / 1.147 ms |

Event detection requires at least one warning in an eligible, pre-estop positive
window within 300 ms of confirmed non-foot contact. Events without such input
are listed separately; they are not silently counted as detected. Warning lead
is therefore capped at 300 ms by this metric. Alarm onsets use a 1-second cooldown.
Exposure is only the observed valid negative-window time, not total run time.

Limitations: same simulator, fixed controller and largely repeated reference
motions. Different episodes may share near-identical prefixes and no-push
trajectories; episode splitting prevents overlapping-window leakage but does
not establish scenario/physical-parameter generalization. Independent, more
diverse held-out simulation and real sensor logs are needed before deployment.
Outcomes include the existing Safety motor-disable response. Balanced-training
sigmoid scores are uncalibrated and are not real-world fall probabilities.
CPU timing excludes ROS and scheduling. No policy/Safety behavior was changed.

See config.json, splits.json, history.json and metrics.json for counts, exact
configuration, confusion matrices, baselines and per-event details. The model
accepts float32 [batch,20,8] in feature order from config.json. Normalization is
included and output is an uncalibrated logit; apply sigmoid then the saved
threshold. model_torchscript.pt is the standalone artifact.
