"""offline.ChainTiming: sampled ROS-chain timing for the offline loop."""
import numpy as np
import pytest

offline = pytest.importorskip('walking_controller.offline')


def test_tables_are_distributions():
    for kw in ({}, offline.DEGRADED_TIMING):
        timing = offline.ChainTiming(**kw)
        for table in (timing.gaps_ms, timing.age_ms, timing.act_ms):
            ms, p = np.array(table).T
            assert np.all(ms >= 0) and abs(p.sum() - 1.0) < 0.01


def test_sampler_matches_table_in_physics_steps():
    draw = offline.ChainTiming(seed=1).sampler(0.005)
    gaps = np.array([draw(offline.ChainTiming.gaps_ms) for _ in range(20000)]) * 5.0
    ms, p = np.array(offline.ChainTiming.gaps_ms).T
    assert gaps.mean() == pytest.approx(float(ms @ p / p.sum()), rel=0.03)


def test_rtf_scales_wall_delays_to_physics_steps():
    # At a real-time factor of 0.5, 20 ms of wall time is 10 ms of physics: 2 steps.
    draw = offline.ChainTiming(rtf=0.5).sampler(0.005)
    assert draw(((20, 1.0),)) == 2


def test_same_seed_same_sequence():
    a, b = (offline.ChainTiming(seed=3).sampler(0.005) for _ in range(2))
    table = offline.ChainTiming.act_ms
    assert [a(table) for _ in range(50)] == [b(table) for _ in range(50)]
