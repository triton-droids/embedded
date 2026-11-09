"""Velocity input (cmd_vel_topic, e.g. /ball_chase/cmd_vel) -> step length / turn per step."""
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

node = pytest.importorskip('walking_controller.node')
N = node.WalkingControllerNode


def fake(max_step=0.06, max_turn=0.08, timeout=0.5):
    p = SimpleNamespace(t_ds=0.2, t_ss=0.3, step_length_m=0.0, turn_rad=0.0)
    obj = SimpleNamespace(controller=SimpleNamespace(p=p), cmd_vel=None,
                          params={'cmd_vel_max_step_m': max_step, 'cmd_vel_max_turn_rad': max_turn,
                                  'cmd_vel_timeout_s': timeout},
                          get_logger=lambda: Mock())
    for name in ('step_period_s', '_on_cmd_vel', '_check_cmd_vel'):
        setattr(obj, name, MethodType(getattr(N, name), obj))
    obj._clamp = N._clamp
    return obj


def twist(v, w):
    return SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w)))


def test_velocity_is_converted_per_step_and_capped():
    obj = fake()
    obj._on_cmd_vel(twist(0.08, 0.10))                 # 0.5 s per step
    assert obj.controller.p.step_length_m == pytest.approx(0.04)
    assert obj.controller.p.turn_rad == pytest.approx(0.05)
    obj._on_cmd_vel(twist(0.20, -0.50))                # ball chase maxima
    assert obj.controller.p.step_length_m == pytest.approx(0.06)
    assert obj.controller.p.turn_rad == pytest.approx(-0.08)
    obj._on_cmd_vel(twist(-0.2, 0.0))                  # no backward steps
    assert obj.controller.p.step_length_m == 0.0


def test_turn_cap_never_exceeds_the_node_limit():
    obj = fake(max_turn=1.0)
    obj._on_cmd_vel(twist(0.0, 2.0))
    assert obj.controller.p.turn_rad == pytest.approx(node.MAX_TURN_RAD)


def test_non_finite_velocity_is_ignored():
    obj = fake()
    obj._on_cmd_vel(twist(0.08, 0.10))
    obj._on_cmd_vel(twist(float('nan'), 0.0))
    assert obj.controller.p.step_length_m == pytest.approx(0.04)


def test_stale_velocity_stops(monkeypatch):
    obj = fake(timeout=0.5)
    clock = [100.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    obj._on_cmd_vel(twist(0.08, 0.10))
    clock[0] += 0.4
    obj._check_cmd_vel()
    assert obj.controller.p.step_length_m == pytest.approx(0.04)
    clock[0] += 0.2
    obj._check_cmd_vel()
    assert obj.controller.p.step_length_m == 0.0 and obj.controller.p.turn_rad == 0.0
    assert obj.cmd_vel is None
