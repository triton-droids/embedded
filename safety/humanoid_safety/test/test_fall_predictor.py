"""Advisory fall predictor: features, gating, confirmation and model parity."""
import json
import math
from pathlib import Path

import pytest

from humanoid_safety.fall_predictor.core import (FallPredictor, Head, ModelSpec, imu_features,
                                                 load_spec)

PACKAGE = Path(__file__).resolve().parents[1]
SPEC_PATH = PACKAGE / 'models' / 'fall_predictor' / 'dual_v2_trial03.json'
SEGMENT = Path(__file__).with_name('data') / 'fall_predictor_shadow_segment.json'
STEP_NS = 10_000_000  # 100 Hz, the training and BNO085 rate


def spec(history=3, confirm=2, threshold=0.5):
    return ModelSpec('test', '', '', 'x', history, ('f',), 0.005, 0.02, 1.0,
                     (Head('a', 0.3, threshold, confirm),))


class FixedScorer:
    def __init__(self, scores):
        self.scores = list(scores)
        self.calls = 0

    def __call__(self, window):
        self.calls += 1
        return [self.scores.pop(0)]


def feed(predictor, count, start_ns=0, step_ns=STEP_NS, estop=False):
    out = []
    for i in range(count):
        out.append(predictor.update(start_ns + i * step_ns, [0.0], estop))
    return out


def test_features_match_training_collector():
    half = math.radians(10) / 2
    roll_only = (math.cos(half), math.sin(half), 0.0, 0.0)
    f = imu_features(roll_only, (0.1, 0.2, 0.3), (0.0, 0.0, 9.81))
    assert f[0] == pytest.approx(math.radians(10))
    assert f[1] == pytest.approx(0.0, abs=1e-12)
    assert f[2:] == [0.1, 0.2, 0.3, 0.0, 0.0, 9.81]
    # Gimbal-limit input stays finite (asin argument is clamped).
    assert math.isfinite(imu_features((0.7072, 0.0, 0.7072, 0.0), (0, 0, 0), (0, 0, 0))[1])


def test_window_fills_before_scoring_and_confirmation_needs_consecutive_frames():
    scorer = FixedScorer([0.9, 0.9, 0.1, 0.9, 0.9])
    p = FallPredictor(spec(history=3, confirm=2), scorer)
    results = feed(p, 7)
    assert results[:2] == [None, None]
    alarms = [r.alarms['a'] for r in results[2:]]
    assert alarms == [False, True, False, False, True]
    assert [r.onsets for r in results[2:]] == [[], ['a'], [], [], ['a']]


def test_estop_in_window_blocks_alarm_until_it_leaves_the_window():
    p = FallPredictor(spec(history=3, confirm=1), FixedScorer([0.9] * 10))
    p.update(0, [0.0], False)
    p.update(STEP_NS, [0.0], True)
    blocked = [p.update(k * STEP_NS, [0.0], False) for k in (2, 3)]
    assert [r.eligible for r in blocked] == [False, False]
    assert not any(r.alarms['a'] for r in blocked)
    assert p.update(4 * STEP_NS, [0.0], False).alarms['a']  # estop sample has left


def test_stamp_gaps_outside_training_range_are_not_eligible():
    p = FallPredictor(spec(history=3, confirm=1), FixedScorer([0.9] * 10))
    results = feed(p, 3, step_ns=30_000_000)  # 33 Hz: too slow for the model
    assert results[-1].eligible is False and results[-1].alarms['a'] is False


def test_stamp_jump_or_reorder_discards_history():
    p = FallPredictor(spec(history=3, confirm=1), FixedScorer([0.9] * 10))
    feed(p, 2)
    assert p.update(5 * 10**9, [0.0], False) is None  # >= reset_gap_s later
    assert p.resets == 1 and len(p.window) == 1
    assert p.update(4 * 10**9, [0.0], False) is None  # backwards
    assert p.resets == 2


def test_spec_rejects_invalid_policy(tmp_path):
    raw = json.loads(SPEC_PATH.read_text())
    raw['heads'][0]['confirm_frames'] = 0
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        load_spec(bad)


def test_installed_model_reproduces_online_shadow_scores_and_alarms():
    """ONNX + core replay of a recorded fall: same scores and alarm onset as TorchScript online."""
    pytest.importorskip('onnxruntime')
    from humanoid_safety.fall_predictor.core import OnnxScorer
    model_spec = load_spec(SPEC_PATH)
    predictor = FallPredictor(model_spec, OnnxScorer(model_spec))
    samples = json.loads(SEGMENT.read_text())['samples']
    onset = None
    for i, s in enumerate(samples):
        result = predictor.update(s['stamp_ns'], s['f'], False)
        if i < model_spec.history - 1:
            assert result is None
            continue
        assert result.scores['300ms'] == pytest.approx(s['p300'], abs=1e-4)
        assert result.scores['600ms'] == pytest.approx(s['p600'], abs=1e-4)
        if i >= 25:  # both run counters are back at zero in the recording by here
            assert result.alarms['300ms'] == s['alarm300']
            assert result.alarms['600ms'] == s['alarm600']
        if '600ms' in result.onsets:
            onset = i
    assert onset == 70
