import pytest
from humanoid_ball_chase.last_ball import LastBall

def seen(lock, t, xy=(1., 2.)):
    for dt in (0., .11, .22):
        lock.observe(t+dt, xy, .02, .9)

def test_continuous_updates_then_keeps_last_real_image():
    ball = LastBall()
    seen(ball, 1.)
    ball.observe(1.33, (1.02, 2.), .02, .9)
    assert ball.point == (1.02, 2.)
    assert ball.usable(5.)
    assert ball.point == (1.02, 2.)
    assert not ball.usable(14.)

def test_outlier_stops_use_until_consistent_again():
    ball = LastBall()
    seen(ball, 1.)
    ball.observe(1.33, (2., 3.), .02, .9)
    assert not ball.usable(1.34)
    seen(ball, 1.44, (1., 2.))
    assert ball.usable(1.7)

def test_bad_quality_and_old_frames_do_not_replace_target():
    ball = LastBall()
    seen(ball, 1.)
    ball.observe(1.1, (8., 9.), .02, .9)
    assert ball.point == (1., 2.)
    ball.observe(1.4, (8., 9.), .2, .9)
    assert not ball.usable(1.5)

def test_single_sighting_cannot_authorize_target():
    ball = LastBall()
    ball.observe(1., (1., 2.), .02, .9)
    assert not ball.usable(1.1)
    assert not ball.close_seen


def test_low_confidence_does_not_invalidate_or_refresh_last_good_position():
    ball = LastBall()
    seen(ball, 1.)
    point, stamp = ball.point, ball.stamp
    for t in (1.4, 2., 5., 12.):
        ball.observe(t, (8., 9.), .02, .4)
        assert ball.point == point
        assert ball.stamp == stamp
        assert ball.usable(t)
    ball.observe(14., (8., 9.), .02, .4)
    assert not ball.usable(14.)
    assert ball.stamp == stamp


def test_low_confidence_cannot_establish_an_initial_target():
    ball = LastBall()
    for t in (1., 1.11, 1.22):
        ball.observe(t, (1., 2.), .02, .49)
    assert ball.point is None
    assert not ball.usable(1.3)


def test_reacquisition_gap_keeps_consistent_last_anchor_without_refreshing_it():
    ball=LastBall()
    seen(ball,1.)
    old_stamp=ball.stamp
    ball.observe(2.,(1.005,2.),.02,.9)
    assert ball.usable(2.01)
    assert ball.stamp==old_stamp
    assert ball.point==(1.,2.)
    ball.observe(2.11,(1.005,2.),.02,.9)
    ball.observe(2.22,(1.005,2.),.02,.9)
    assert ball.stamp==2.22


def test_contradictory_new_image_after_gap_still_stops_old_target():
    ball=LastBall()
    seen(ball,1.)
    ball.observe(2.,(2.,3.),.02,.9)
    assert not ball.usable(2.01)


def test_one_image_after_expiry_cannot_revive_old_target():
    ball=LastBall()
    seen(ball,1.)
    ball.observe(15.,(1.,2.),.02,.9)
    assert not ball.usable(15.01)
