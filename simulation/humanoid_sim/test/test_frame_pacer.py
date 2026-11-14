"""Camera frame pacing in simulated time."""
import pytest

from humanoid_sim.frame_pacer import FramePacer

MS = 1_000_000


def test_sim_clock_renders_one_frame_per_simulated_period():
    pacer = FramePacer(0.1, sim_clock=True)
    stamps = [0, 5*MS, 95*MS, 100*MS, 150*MS, 205*MS, 290*MS, 300*MS]
    assert [pacer.due(s) for s in stamps] == [True, False, False, True, False, True, False, True]


def test_sim_clock_restarts_after_a_jump_without_bursting():
    pacer = FramePacer(0.1, sim_clock=True)
    assert pacer.due(0)
    assert pacer.due(10_000*MS)          # Long pause: one frame, not a hundred.
    assert not pacer.due(10_050*MS)
    assert pacer.due(10_100*MS)


def test_wall_mode_always_renders_and_rejects_bad_period():
    assert FramePacer(0.1, sim_clock=False).due(None)
    assert not FramePacer(0.1, sim_clock=True).due(None)
    with pytest.raises(ValueError):
        FramePacer(0.0, sim_clock=True)
