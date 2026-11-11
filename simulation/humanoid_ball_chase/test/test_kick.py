import math

from humanoid_ball_chase.kick import KickApproach, KickConfig, WalkState


def test_far_or_moving_ball_leaves_the_chase_in_charge():
    k = KickApproach()
    walk = WalkState()
    assert not k.update(0.0, (2.0, 0.0), 0.0, 0.02, walk).active
    assert not k.update(0.1, (0.5, 0.0), 0.5, 0.02, walk).active   # rolling
    assert not k.update(0.2, (0.5, 0.0), 0.0, 0.5, walk).active    # uncertain
    assert k.state == 'chase'


def test_turns_to_put_the_ball_in_front_of_the_nearer_foot():
    k = KickApproach()
    d = k.update(0.0, (0.6, -0.02), 0.0, 0.02, WalkState())
    # The model's 'left_foot' is on the robot's right (y = -0.15).
    assert d.active and k.foot == 'left_foot'
    # The ball should appear further right -> turn left so it moves right in base_link (wz > 0).
    assert d.wz > 0.0 and d.vx == 0.0  # bearing error above turn_first: turn only
    want = -math.asin(0.15 / 0.6)
    d = k.update(0.1, (0.6 * math.cos(want), 0.6 * math.sin(want)), 0.0, 0.02, WalkState())
    assert abs(d.wz) < 1e-6 and 0.0 < d.vx <= KickConfig().max_speed


def test_window_settle_kick_cooldown():
    k = KickApproach()
    walk = WalkState(standing=False, kicks=3)
    ball = (0.20, 0.15)
    d = k.update(0.0, ball, 0.0, 0.02, walk)
    assert k.foot == 'right_foot' and k.state == 'settle' and (d.vx, d.wz) == (0.0, 0.0)
    d = k.update(0.5, ball, 0.0, 0.02, walk)
    assert d.kick_foot is None and k.state == 'settle'
    walk.standing = True
    d = k.update(1.0, ball, 0.0, 0.02, walk)
    assert d.kick_foot == 'right_foot' and k.state == 'kick'
    assert k.update(1.1, ball, 0.0, 0.02, walk).kick_foot is None  # sent once
    walk.kick = 'kicking'
    assert k.update(1.3, None, 0.0, 1.0, walk).active  # ball out of view while kicking: keep still
    walk.kick, walk.kicks = None, 4
    k.update(1.6, None, 0.0, 1.0, walk)
    assert k.state == 'cooldown' and k.kicked == 1
    assert k.update(2.0, (1.5, 0.0), 0.0, 0.02, walk).active
    assert not k.update(3.7, (1.5, 0.0), 0.0, 0.02, walk).active and k.state == 'chase'


def test_ball_leaving_the_window_goes_back_to_approach_and_overrun_gives_up():
    k = KickApproach()
    walk = WalkState()
    k.update(0.0, (0.20, -0.15), 0.0, 0.02, walk)
    assert k.state == 'settle'
    k.update(0.2, (0.40, -0.15), 0.0, 0.02, walk)
    assert k.state == 'approach'
    k.update(0.4, (0.05, -0.15), 0.0, 0.02, walk)
    assert k.state == 'cooldown'


def test_foot_choice_has_hysteresis():
    k = KickApproach()
    walk = WalkState()
    k.update(0.0, (0.7, 0.01), 0.0, 0.02, walk)
    assert k.foot == 'right_foot'   # on the robot's left in this model
    k.update(0.1, (0.7, -0.05), 0.0, 0.02, walk)
    assert k.foot == 'right_foot'
    k.update(0.2, (0.7, -0.10), 0.0, 0.02, walk)
    assert k.foot == 'left_foot'


def test_bad_estimate_while_engaged_holds_still_then_gives_up():
    k = KickApproach()
    walk = WalkState()
    k.update(0.0, (0.6, 0.0), 0.0, 0.02, walk)
    assert k.state == 'approach'
    d = k.update(0.1, (0.6, 0.0), 0.2, 0.02, walk)  # velocity noise below the keep limit: still approaching
    assert d.active and k.state == 'approach'
    d = k.update(0.2, None, 0.0, 1.0, walk)
    assert d.active and (d.vx, d.wz) == (0.0, 0.0) and k.state == 'approach'
    k.update(1.5, None, 0.0, 1.0, walk)
    assert k.state == 'cooldown'
