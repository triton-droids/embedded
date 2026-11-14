"""Start-up gyro bias calibration."""
import numpy as np
import pytest

from humanoid_sim.gyro_bias import GyroBiasCalibrator

MS = 1_000_000
BIAS = np.array([0.003, -0.002, 0.0015])


def test_off_passes_samples_through():
    calibrator = GyroBiasCalibrator(0.0)
    assert calibrator.update(0, [1.0, 2.0, 3.0])
    assert calibrator.correct([1.0, 2.0, 3.0]) == pytest.approx([1.0, 2.0, 3.0])


def test_mean_over_a_still_window_is_subtracted():
    rng = np.random.default_rng(0)
    calibrator = GyroBiasCalibrator(2.0)
    ready_at = None
    for i in range(1000):
        if calibrator.update(i * 5 * MS, BIAS + rng.normal(0, 0.0025, 3)) and ready_at is None:
            ready_at = i * 5 * MS
    assert ready_at == 2000 * MS
    # Standard error 0.0025 / sqrt(401) = 1.25e-4 per axis; 4 sigma.
    assert calibrator.bias == pytest.approx(BIAS, abs=5e-4)
    assert calibrator.correct(BIAS) == pytest.approx(np.zeros(3), abs=5e-4)
    assert not calibrator.failed


def test_motion_restarts_the_window():
    calibrator = GyroBiasCalibrator(1.0)
    stamps = iter(range(0, 10**10, 5 * MS))
    for _ in range(150):                      # 0.75 s still
        assert not calibrator.update(next(stamps), BIAS)
    calibrator.update(next(stamps), [0.5, 0.0, 0.0])  # head moves
    for _ in range(200):                      # 0.995 s still again: not yet
        assert not calibrator.update(next(stamps), BIAS)
    assert calibrator.update(next(stamps), BIAS)
    assert calibrator.bias == pytest.approx(BIAS)


def test_gives_up_without_a_still_window():
    calibrator = GyroBiasCalibrator(2.0, timeout_s=3.0)
    ready = [calibrator.update(i * 5 * MS, [0.3, 0.0, 0.0]) for i in range(601)]
    assert not any(ready[:600]) and ready[600]
    assert calibrator.failed and calibrator.correct([0.3, 0, 0]) == pytest.approx([0.3, 0, 0])


def test_rejects_bad_settings():
    with pytest.raises(ValueError):
        GyroBiasCalibrator(-1.0)
    with pytest.raises(ValueError):
        GyroBiasCalibrator(5.0, timeout_s=5.0)
