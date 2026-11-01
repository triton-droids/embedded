import numpy as np
import pytest

from pathlib import Path
import hashlib
import json

from prepare import REGISTRY, SOURCE, prepare, warp

NAMES = [f'{side}_{joint}_joint' for side in ('left', 'right')
         for joint in ('hip1', 'hip2', 'thigh', 'knee', 'ankle')]


def test_identity_and_unselected_axes_are_exact():
    q = np.arange(800, dtype=np.float32).reshape(80, 10)*0.001
    dq = np.ones_like(q)*0.05
    a, b = warp(q, dq, NAMES, 1.0)
    np.testing.assert_array_equal(a, q)
    np.testing.assert_array_equal(b, dq)
    a, b = warp(q, dq, NAMES, 0.95)
    np.testing.assert_array_equal(a[0], q[0])
    np.testing.assert_array_equal(b[0], dq[0])
    np.testing.assert_array_equal(a[:, [1, 2, 6, 7]], q[:, [1, 2, 6, 7]])
    np.testing.assert_array_equal(b[:, [1, 2, 6, 7]], dq[:, [1, 2, 6, 7]])
    np.testing.assert_allclose(a[60, 0]-q[0, 0], 0.95*(q[60, 0]-q[0, 0]))


def test_velocity_contains_ramp_derivative():
    # Analytic linear motion; compare warped velocity to its numerical derivative.
    t = np.arange(100, dtype=np.float64)/50
    q = np.repeat(t[:, None], 10, axis=1)
    dq = np.ones_like(q)
    a, b = warp(q, dq, NAMES, 1.05)
    np.testing.assert_allclose(np.gradient(a[:, 0], 0.02)[2:-2], b[2:-2, 0], atol=0.0002)


@pytest.mark.parametrize('scale', [0.0, 1.2, float('nan'), float('inf')])
def test_rejects_large_or_invalid_scale(scale):
    with pytest.raises(ValueError):
        warp(np.zeros((60, 10)), np.zeros((60, 10)), NAMES, scale)


def test_real_export_preserves_source_and_changes_policy_response():
    from motor_control_hybrid.tracking_onnx import TrackingPolicy
    original = SOURCE/'20260914_170827.onnx'
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    before = digest(original), digest(REGISTRY)
    output = Path(__file__).resolve().parent/'generated'/'test_export'
    path, registry, manifest = prepare(0.95, output)
    assert (digest(original), digest(REGISTRY)) == before
    baseline, modified = TrackingPolicy(original), TrackingPolicy(path)
    assert manifest['neural_weights_unchanged']
    assert 0 < manifest['max_reference_delta_rad'] < 0.04
    for frame in (0, 100):
        args = (frame, np.zeros(3), np.array([0, 0, -1]), baseline.reference_q[frame],
                baseline.reference_dq[frame], np.zeros(10))
        a = baseline.run(baseline.observation(*args), frame)[1]
        b = modified.run(modified.observation(*args), frame)[1]
        if frame == 0:
            np.testing.assert_array_equal(a, b)
        else:
            assert np.max(np.abs(a-b)) > 0.001
            assert np.all(b >= modified.target_limits[0])
            assert np.all(b <= modified.target_limits[1])


def test_output_cannot_overwrite_production(tmp_path):
    with pytest.raises(ValueError, match='Output must'):
        prepare(1.0, tmp_path/'outside_experiment')
