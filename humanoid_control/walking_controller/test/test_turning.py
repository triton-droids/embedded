"""Turning (StepParams.turn_rad, /walk/turn): offline closed loop.

Each swing target is rotated by the turn about the stance foot, the MPC frame
follows the measured stance foot, the swing foot's yaw is interpolated from its
start to the target by s = swing_yaw_end, and the torso yaw follows the frame.
"""
import math

import numpy as np
import pytest

from conftest import GAINS


def base_yaw(q):
    w, x, y, z = q[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def turning(model, schedule, duration, delay_steps=2):
    """schedule: [(t, turn_rad, step_length_m)]. Returns controller, result,
    unwrapped base yaw change and displacement after the first command."""
    from walking_controller.offline import run
    from walking_controller.stepping import StepParams, SteppingController
    from walking_controller.wbc import WBCGains
    paths, robot, limits = model
    ctl = SteppingController(robot, limits.torque, StepParams(gains=WBCGains(**GAINS)))
    state = {'yaw': 0.0, 'prev': None}

    def on_step(t, sim, c):
        y = base_yaw(sim.data.qpos)
        if state['prev'] is not None:
            state['yaw'] += math.atan2(math.sin(y - state['prev']), math.cos(y - state['prev']))
        state['prev'] = y
        for ts, turn, length in schedule:
            if abs(t - ts) < sim.dt / 2:
                c.p.turn_rad, c.p.step_length_m = turn, length
                if 'start' not in state:
                    state['start'] = (state['yaw'], sim.data.qpos[:2].copy(), c.steps)
        state['end'] = (state['yaw'], sim.data.qpos[:2].copy(), getattr(c, 'steps', 0))
        return False

    res = run(ctl, duration, delay_steps=delay_steps, paths=paths, robot=robot, limits=limits,
              on_step=on_step)
    (y0, p0, s0), (y1, p1, s1) = state['start'], state['end']
    return ctl, res, y1 - y0, float(np.linalg.norm(p1 - p0)), s1 - s0


@pytest.mark.parametrize('turn', [0.10, -0.10])
def test_turns_in_place_both_ways(model, turn):
    ctl, res, yaw, moved, steps = turning(model, [(5.0, turn, 0.0)], 17.0)
    assert not res['fault'] and not res['fallen'], res['fault']
    assert steps >= 20
    assert yaw * np.sign(turn) > math.radians(90)                # M5: a 90 deg turn in place
    assert abs(yaw) / steps > 0.75 * abs(turn), math.degrees(yaw) / steps
    assert moved < 0.3
    # The walking frame follows the feet: its heading matches the base yaw.
    assert abs(ctl.heading - yaw) < math.radians(15)


def test_walks_straight_after_a_large_turn(model):
    """Regression: the sole-centre offset was a fixed world vector, so after
    ~150 deg the support regions were off and walking after the turn fell
    within ~4 s; it is now rotated with the foot's yaw."""
    ctl, res, yaw, moved, steps = turning(model, [(5.0, 0.15, 0.0), (14.0, 0.0, 0.06)], 21.0)
    assert not res['fault'] and not res['fallen'], res['fault']
    assert yaw > math.radians(120)
    assert moved > 0.25


def test_walks_an_arc(model):
    ctl, res, yaw, moved, steps = turning(model, [(5.0, 0.10, 0.05)], 15.0)
    assert not res['fault'] and not res['fallen'], res['fault']
    assert yaw > math.radians(90) and moved > 0.15


@pytest.mark.slow
@pytest.mark.parametrize('delay_steps', [3, 4])
def test_chase_pattern_49_s(model, delay_steps):
    """Turn ~90 deg, walk 3 s, turn back, ... at 15 / 20 ms: how ball chase uses it."""
    schedule, t = [], 5.0
    for k in range(4):
        schedule += [(t, 0.10 if k % 2 == 0 else -0.10, 0.0), (t + 8.0, 0.0, 0.06)]
        t += 11.0
    _, res, _, moved, _ = turning(model, schedule, 49.0, delay_steps=delay_steps)
    assert not res['fault'] and not res['fallen'], res['fault']
    assert moved > 0.5


@pytest.mark.slow
@pytest.mark.parametrize('turn', [0.10, 0.15, -0.15])
def test_continuous_turn_49_s_at_15_ms(model, turn):
    """Before the sole-centre fix continuous turns >= 0.10 rad/step fell after
    20-33 s (mostly to the left): the stance widened and the hips rolled."""
    _, res, yaw, moved, _ = turning(model, [(5.0, turn, 0.0)], 49.0, delay_steps=3)
    assert not res['fault'] and not res['fallen'], res['fault']
    assert abs(yaw) > math.radians(400) and moved < 0.5
