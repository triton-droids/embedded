"""Velocity input (cmd_vel_topic, e.g. /ball_chase/cmd_vel) -> step length / turn per step."""
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

node = pytest.importorskip('walking_controller.node')
N = node.WalkingControllerNode


class FakeController:
    def __init__(self):
        self.p = SimpleNamespace(t_ds=0.2, t_ss=0.3, step_length_m=0.0, turn_rad=0.0)
        self.stop_requested, self.resumed = False, 0

    def request_stop(self):
        self.stop_requested = True

    def resume(self):
        self.stop_requested = False
        self.resumed += 1


def fake(max_step=0.06, max_turn=0.08, timeout=0.5, turn_free=0.0, turn_stop=0.0, stand_after=0.0):
    obj = SimpleNamespace(controller=FakeController(), cmd_vel=None, _rest_since=None,
                          params={'cmd_vel_max_step_m': max_step, 'cmd_vel_max_turn_rad': max_turn,
                                  'cmd_vel_timeout_s': timeout, 'cmd_vel_turn_free_rad': turn_free,
                                  'cmd_vel_turn_stop_rad': turn_stop, 'cmd_vel_stand_after_s': stand_after,
                                  'cmd_vel_rest_step_m': 0.005, 'cmd_vel_rest_turn_rad': 0.01},
                          get_logger=lambda: Mock())
    for name in ('step_period_s', '_on_cmd_vel', '_check_cmd_vel', '_update_stand'):
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


def test_turn_first_shortens_the_step_while_turning():
    obj = fake(turn_free=0.04, turn_stop=0.10)
    obj._on_cmd_vel(twist(0.20, 0.05))                 # turn 0.025: full step
    assert obj.controller.p.step_length_m == pytest.approx(0.06)
    obj._on_cmd_vel(twist(0.20, 0.50))                 # turn at the 0.08 cap: a third
    assert obj.controller.p.turn_rad == pytest.approx(0.08)
    assert obj.controller.p.step_length_m == pytest.approx(0.02)
    obj._on_cmd_vel(twist(0.20, -0.14))                # turn -0.07: half
    assert obj.controller.p.step_length_m == pytest.approx(0.03)


def test_default_parameters_turn_first():
    src = open(node.__file__).read()
    assert "'cmd_vel_turn_free_rad': 0.04" in src and "'cmd_vel_turn_stop_rad': 0.10" in src


def test_rest_command_stands_after_the_delay_and_motion_walks_again(monkeypatch):
    obj = fake(stand_after=1.0)
    clock = [100.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    c = obj.controller
    obj._on_cmd_vel(twist(0.0, 0.01))                  # turn 0.005/step: at rest
    clock[0] += 0.9
    obj._on_cmd_vel(twist(0.0, -0.01))
    assert not c.stop_requested
    clock[0] += 0.2
    obj._on_cmd_vel(twist(0.0, 0.0))
    assert c.stop_requested and c.p.step_length_m == 0.0 and c.p.turn_rad == 0.0
    obj._on_cmd_vel(twist(0.0, 0.10))                  # turn 0.05/step: walk (turn) again
    assert not c.stop_requested and c.resumed == 1
    assert c.p.turn_rad == pytest.approx(0.05)
    clock[0] += 2.0                                    # the rest timer restarted
    obj._on_cmd_vel(twist(0.0, 0.0))
    assert not c.stop_requested
    obj._on_cmd_vel(twist(0.05, 0.0))
    assert c.resumed == 1                              # resume only when standing


def test_stale_velocity_also_stands(monkeypatch):
    obj = fake(timeout=0.5, stand_after=1.0)
    clock = [100.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    obj._on_cmd_vel(twist(0.1, 0.0))
    for _ in range(20):                                # 2 s of 0.1 s checks
        clock[0] += 0.1
        obj._check_cmd_vel()
    assert obj.cmd_vel is None and obj.controller.stop_requested


def test_stand_disabled_keeps_stepping_in_place(monkeypatch):
    obj = fake(stand_after=0.0)
    clock = [100.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    for _ in range(30):
        obj._on_cmd_vel(twist(0.0, 0.0))
        clock[0] += 0.1
    assert not obj.controller.stop_requested


def test_default_stands_after_one_second():
    src = open(node.__file__).read()
    assert "'cmd_vel_stand_after_s': 1.0" in src
