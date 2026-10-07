"""Observation contract, frame conventions, joint mapping and stale-data gating."""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('onnxruntime')
pytest.importorskip('onnx')
from motor_control_hybrid.tracking_onnx import (  # noqa: E402
    GravityEstimator, TrackingPolicy, fresh, joint_feedback)


@pytest.fixture(scope='module')
def policy():
    model = (Path.home() / 'Github/simulation/logs/legs_tracking/'
             '20260914_170827/20260914_170827.onnx')
    if not model.exists():
        pytest.skip('Tracking export not installed; model-independent tests still run')
    return TrackingPolicy(model)


def test_observation_matches_tracking_export(policy):
    q = np.arange(10, dtype=np.float32) * 0.01
    dq = q + 0.3
    previous = q + 0.6
    obs = policy.observation(17, np.array([0.1, 0.2, 0.3]),
                             np.array([0, 0, -1]), q, dq, previous)
    np.testing.assert_array_equal(obs[0, :10], policy.reference_q[17])
    np.testing.assert_array_equal(obs[0, 10:20], policy.reference_dq[17])
    np.testing.assert_allclose(obs[0, 20:23], [0.1, 0.2, 0.3])
    np.testing.assert_array_equal(obs[0, 23:33], q - policy.offset)
    np.testing.assert_array_equal(obs[0, 33:43], dq)
    np.testing.assert_array_equal(obs[0, 43:53], previous)
    np.testing.assert_array_equal(obs[0, 53:], [0, 0, -1])
    assert obs.dtype == np.float32
    action, target = policy.run(obs, 17)
    np.testing.assert_allclose(target, action * 0.2, rtol=1e-6)
    assert policy.frames == 299


def test_invalid_observation_is_rejected(policy):
    with pytest.raises(ValueError):
        policy.observation(0, np.array([float('nan'), 0, 0]), np.array([0, 0, -1]),
                           np.zeros(10), np.zeros(10), np.zeros(10))


def test_feedback_reorders_by_name_and_rejects_partial_or_nonfinite():
    q, dq = joint_feedback(['b', 'a'], [2, 1], [20, 10], ['a', 'b'])
    np.testing.assert_array_equal(q, [1, 2])
    np.testing.assert_array_equal(dq, [10, 20])
    for names, positions, velocities in [
            (['a'], [1], [10]), (['a', 'b'], [1, 2], []),
            (['a', 'a'], [1, 2], [0, 0]),
            (['a', 'b'], [1, float('nan')], [0, 0])]:
        with pytest.raises(ValueError):
            joint_feedback(names, positions, velocities, ['a', 'b'])


def test_freshness_checks_receipt_and_source_timestamp():
    assert fresh(9.99, 999.99, 10, 1000, 0.1)
    assert not fresh(None, None, 10, 1000, 0.1)
    assert not fresh(9.8, 999.99, 10, 1000, 0.1)
    assert not fresh(9.99, 999.8, 10, 1000, 0.1)
    assert not fresh(9.99, 1000.1, 10, 1000, 0.1)


def test_gravity_down_and_tilted_sensor():
    estimator = GravityEstimator()
    np.testing.assert_allclose(estimator.update(np.array([0, 0, 9.80665]),
                                                np.zeros(3), 1), [0, 0, -1])
    np.testing.assert_allclose(estimator.update(np.array([9.80665, 0, 0]),
                                                np.zeros(3), 2), [-1, 0, 0])
    with pytest.raises(ValueError):
        estimator.update(np.zeros(3), np.zeros(3), 3)
