import math

import numpy as np
import pytest

from humanoid_ball_chase import world_model as wm
from humanoid_ball_chase.tracking.tracker import COASTING, LOST, TENTATIVE, TRACKING, BallTracker, TrackOutput


def out(status, x, pos_std=0.05, vel_std=0.05, last_seen=0.0):
    P = np.diag([pos_std**2, pos_std**2, vel_std**2, vel_std**2])
    return TrackOutput(status, np.asarray(x, float), P, last_seen, 0.8, status == TRACKING)


def test_ball_center_moves_one_radius_along_the_ray():
    c = wm.ball_center([0.3, 0.4, 1.2], 0.1)
    assert np.linalg.norm(c) == pytest.approx(np.linalg.norm([0.3, 0.4, 1.2]) + 0.1)
    assert np.cross(c, [0.3, 0.4, 1.2]) == pytest.approx(np.zeros(3))
    assert wm.ball_center([0.3, 0.4, 1.2], 0.0) == pytest.approx([0.3, 0.4, 1.2])


def test_config_is_retuned_for_6hz_and_rejects_unknown_keys():
    cfg = wm.tracker_config(friction='0.7')
    assert cfg.friction == pytest.approx(0.7)
    assert cfg.confirm_hits == 2 and cfg.tentative_timeout > 2 / 6.0
    with pytest.raises(ValueError):
        wm.tracker_config(nope=1)


def test_plan_states():
    assert wm.plan(None, 0.4).mode == 'none'
    assert wm.plan(out(TENTATIVE, [1, 0, 0, 0]), 0.4).mode == 'none'
    assert wm.plan(out(LOST, [1, 0, 0, 0]), 0.4).mode == 'none'
    t = wm.plan(out(TRACKING, [1, 0, 0, 0]), 0.4)
    assert t.mode == 'tracking' and t.speed_scale == 1.0 and t.point_odom == pytest.approx([1, 0])
    # Between frames peek() says COASTING; a recent detection still counts as tracking.
    assert wm.plan(out(COASTING, [1, 0, 0, 0], last_seen=10.0), 0.4, now=10.3).mode == 'tracking'
    c = wm.plan(out(COASTING, [1, 0, 0, 0], last_seen=10.0), 0.4, now=11.0)
    assert c.mode == 'coasting' and c.speed_scale == pytest.approx(0.5)
    assert wm.plan(out(COASTING, [1, 0, 0, 0], pos_std=0.8), 0.4, now=99.0).speed_scale == 0.0
    nan = out(TRACKING, [math.nan, 0, 0, 0])
    assert wm.plan(nan, 0.4).mode == 'none'


def test_plan_leads_a_confidently_moving_ball_by_a_short_horizon():
    c = 0.5
    moving = wm.plan(out(TRACKING, [1, 0, 0.4, 0.0]), c)
    gain = (1 - math.exp(-c * 1.0)) / c  # distance per m/s in 1 s with friction
    assert moving.lead and moving.point_odom == pytest.approx([1 + 0.4 * gain, 0.0])
    assert wm.plan(out(TRACKING, [1, 0, 0.4, 0.0]), 0.0).point_odom == pytest.approx([1.4, 0.0])
    unsure = wm.plan(out(TRACKING, [1, 0, 0.4, 0.0], vel_std=0.5), c)
    assert not unsure.lead and unsure.point_odom == pytest.approx([1, 0])
    assert not wm.plan(out(TRACKING, [1, 0, 0.05, 0.0]), c).lead
    capped = wm.plan(out(TRACKING, [0, 0, 3.0, 0.0]), 0.06)
    assert capped.point_odom == pytest.approx([1.0, 0.0])  # max_lead_m
    off = wm.plan(out(TRACKING, [1, 0, 0.4, 0.0]), c, wm.PlanConfig(lead_s=0.0))
    assert not off.lead and off.point_odom == pytest.approx([1, 0])


def test_lead_stays_near_a_ball_that_does_not_stop():
    """A ball pushed in a circle (r 1.5 m, 0.25 m/s) on a low-friction floor: the lead point is
    within 0.3 m of the ball, unlike the rest point v / friction (> 3 m away)."""
    lead = wm.plan(out(TRACKING, [1.5, 0, 0, 0.25]), 0.06)
    assert np.linalg.norm(lead.point_odom - [1.5, 0]) < 0.3


def test_look_pixel_keeps_the_angle_and_handles_points_behind():
    k = np.array([[300., 0, 212], [0, 300., 120], [0, 0, 1]])
    assert wm.look_pixel([0.0, 0.0, 2.0], k) == pytest.approx((212, 120))
    u, v = wm.look_pixel([2.0, -1.0, 1.0], k)  # off image, 63 degrees right
    assert math.atan2(u - 212, 300) == pytest.approx(math.atan(2.0))
    assert math.atan2(v - 120, 300) == pytest.approx(math.atan(-1.0))
    u, _ = wm.look_pixel([-0.5, 0.0, -1.0], k)  # behind, to the left: turn left
    assert u < 212 - 300 * 5


def test_tracker_coasts_through_a_short_occlusion_in_odom():
    """6 Hz, still ball at (2, 0.5); 1 s without detections keeps the estimate in place."""
    tr = BallTracker(wm.tracker_config())
    rng = np.random.default_rng(3)
    t = 0.0
    for _ in range(12):
        t += 1 / 6
        tr.step(t, [wm.measurement(np.array([2.0, 0.5]) + rng.normal(0, 0.02, 2), 2.0, 0.9, 0.03, 0.01)])
    assert tr.status == TRACKING
    for _ in range(6):
        t += 1 / 6
        tr.step(t, [])
    o = tr.peek(t)
    assert o.status == COASTING
    assert o.x[:2] == pytest.approx([2.0, 0.5], abs=0.05)
    assert wm.plan(o, 0.4, now=t).mode == 'coasting'
    for _ in range(12):  # past coast_timeout
        t += 1 / 6
        tr.step(t, [])
    assert tr.peek(t).status == LOST


def test_tracker_velocity_gives_a_lead_point_ahead_of_a_rolling_ball():
    c, v0 = 0.6, np.array([0.0, 0.5])
    tr = BallTracker(wm.tracker_config(friction=c))
    rng = np.random.default_rng(4)
    times = np.arange(1, 9) / 6  # still rolling at 0.22 m/s
    for t in times:
        p = np.array([1.0, 0.0]) + v0 * (1 - math.exp(-c * t)) / c
        tr.step(t, [wm.measurement(p + rng.normal(0, 0.02, 2), 1.5, 0.9, 0.03, 0.01)])
    o = tr.peek(times[-1])
    t = times[-1]
    true_ahead = np.array([1.0, 0.0]) + v0 * (1 - math.exp(-c * (t + 1.0))) / c  # position 1 s later
    chase = wm.plan(o, c, now=t)
    assert chase.lead
    assert chase.point_odom == pytest.approx(true_ahead, abs=0.1)
