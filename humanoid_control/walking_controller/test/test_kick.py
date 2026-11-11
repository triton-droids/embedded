"""Kick (StepParams.kick_*, request_kick): offline MuJoCo + TSID episodes with a ball (tools/kick_eval.py)."""
import pytest


def _episode(model, **kw):
    pytest.importorskip('humanoid_ball_chase')
    from tools.kick_eval import episode
    return episode(**kw)


def _check(r):
    assert not r['fallen'] and not r['fault'], r
    assert r['kicks'] == 1, r
    assert r['ball_peak_speed'] > 0.8 and r['ball_moved_m'] > 1.5, r
    assert abs(r['ball_dir_deg']) < 15.0, r


def test_standing_kick_sends_the_ball_forward(model):
    """Stand, request a right-foot kick: shift, strike, land ahead, square the feet, stand."""
    r = _episode(model, foot='right_foot', ball=True, duration=13.0)
    _check(r)
    names = [e['event'] for e in r['events']]
    assert names[:2] == ['kick_armed', 'kick'], names


@pytest.mark.slow
@pytest.mark.parametrize('seed', [3, 4, 5])
@pytest.mark.parametrize('foot', ['left_foot', 'right_foot'])
def test_standing_kick_with_ros_chain_timing(model, foot, seed):
    """Jetson ROS-chain timing: 0.35 s+ kicks and standing on the staggered landing fell here."""
    from walking_controller.offline import ChainTiming
    _check(_episode(model, foot=foot, ball=True, duration=13.0, timing=ChainTiming(seed=seed)))


@pytest.mark.slow
@pytest.mark.parametrize('lateral', [-0.06, 0.06])
@pytest.mark.parametrize('ahead', [0.12, 0.26])
def test_kick_window(model, ahead, lateral):
    """The ball window the chase's kick approach aims for (ball centre ahead of / beside the sole centre)."""
    r = _episode(model, foot='right_foot', ball=True, ball_ahead=ahead, ball_lateral=lateral, duration=13.0)
    assert not r['fallen'] and not r['fault'] and r['ball_moved_m'] > 1.0, r
