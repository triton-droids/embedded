# Dual-horizon training completed

Outputs: 300 ms and 600 ms logits, with sigmoid(short) <= sigmoid(long).
Normalization is embedded. Apply each saved temporal policy separately.
Old test excluded. New scenario families were assigned before outcomes were known;
only the selected model was evaluated once on the new test fold.
Safety remains unchanged. See config.json, quality.json, best.json and sealed_test_metrics.json.
Labels include the existing controller and Safety response. Same-simulator limits remain.
Metrics from a small independent fold have sampling uncertainty. No deployment was done.
