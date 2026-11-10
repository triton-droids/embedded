import numpy as np
import pytest

from humanoid_sim.image_degradation import DegradationSettings, ImageDegrader, motion_kernel


def frame():
    rng = np.random.default_rng(1)
    return rng.integers(0, 200, (60, 80, 3), dtype=np.uint8)


def test_defaults_return_the_same_frame():
    image = frame()
    assert ImageDegrader(DegradationSettings(), 200.0).apply(image, np.ones(3)) is image


def test_motion_blur_follows_image_flow():
    d = ImageDegrader(DegradationSettings(exposure_s=0.03), 200.0)
    assert d.blur_vector(np.array([0.0, 2.0, 0.0])) == pytest.approx((12.0, 0.0))
    assert d.blur_vector(np.array([1.0, 0.0, 5.0])) == pytest.approx((0.0, 6.0))
    k = motion_kernel(12.0, 0.0)
    assert k.sum() == pytest.approx(1.0)
    assert k[k.shape[0] // 2].sum() == pytest.approx(1.0, abs=0.05)   # horizontal line
    image = np.zeros((40, 40, 3), np.uint8)
    image[:, 20] = 255
    out = d.apply(image, np.array([0.0, 2.0, 0.0]))
    assert out[:, 20, 0].max() < 60 and out[:, 15, 0].min() > 0     # vertical bar smeared sideways
    assert np.array_equal(d.apply(image, np.zeros(3)), image)         # still camera: sharp


def test_exposure_noise_and_jpeg_are_seeded():
    s = DegradationSettings(exposure_gain=1.5, exposure_jitter=0.1, noise_std=4.0, jpeg_quality=40, seed=3)
    a = ImageDegrader(s, 200.0).apply(frame())
    b = ImageDegrader(s, 200.0).apply(frame())
    assert a.dtype == np.uint8 and a.shape == frame().shape
    assert np.array_equal(a, b)
    assert a.mean() > frame().mean() * 1.25
    assert (ImageDegrader(DegradationSettings(exposure_gain=2.0), 1.0).apply(frame()) == 255).mean() > 0.2
