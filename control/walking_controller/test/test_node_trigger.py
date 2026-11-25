"""'state' trigger: one cycle per physics sample, once odom, feet and joints match."""
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

node = pytest.importorskip('walking_controller.node')


def fake(trigger='state', fallback=0.0):
    obj = SimpleNamespace(params={'trigger': trigger, 'trigger_fallback_s': fallback}, inputs={},
                          _last_sample=None, tick=Mock())
    obj._try_tick = MethodType(node.WalkingControllerNode._try_tick, obj)
    return obj


def arrive(obj, name, stamp):
    obj.inputs[name] = (None, stamp, 0.0)
    obj._try_tick()


def test_runs_once_when_the_last_message_of_a_sample_arrives():
    obj = fake()
    arrive(obj, 'odom', 10)
    arrive(obj, 'feet', 10)
    obj.tick.assert_not_called()          # joints of sample 10 not here yet
    arrive(obj, 'joints', 10)
    assert obj.tick.call_count == 1
    arrive(obj, 'joints', 10)             # duplicate: same sample, no second cycle
    assert obj.tick.call_count == 1


def test_joints_first_then_feet_completes_the_sample():
    obj = fake()
    arrive(obj, 'joints', 20)
    arrive(obj, 'odom', 20)
    obj.tick.assert_not_called()
    arrive(obj, 'feet', 20)
    assert obj.tick.call_count == 1


def test_mismatched_samples_are_skipped_not_mixed():
    obj = fake()
    arrive(obj, 'odom', 30)
    arrive(obj, 'feet', 30)
    arrive(obj, 'joints', 25)             # older joint sample: never mix 25 with 30
    obj.tick.assert_not_called()
    arrive(obj, 'odom', 35)
    arrive(obj, 'feet', 35)
    arrive(obj, 'joints', 35)
    assert obj.tick.call_count == 1


def test_timer_trigger_ignores_arrivals():
    obj = fake('timer')
    for name in ('odom', 'feet', 'joints'):
        arrive(obj, name, 40)
    obj.tick.assert_not_called()


def test_fallback_runs_on_latest_after_no_complete_sample(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    obj = fake(fallback=0.015)
    for name in ('odom', 'feet', 'joints'):
        arrive(obj, name, 50)
    assert obj.tick.call_count == 1
    # Under load: joints and odom/feet from alternating samples, never matching.
    clock[0] += 0.005; arrive(obj, 'joints', 55)
    clock[0] += 0.005; arrive(obj, 'odom', 60); arrive(obj, 'feet', 60)
    assert obj.tick.call_count == 1         # within the fallback time: wait
    clock[0] += 0.010; arrive(obj, 'joints', 65)
    assert obj.tick.call_count == 2         # 20 ms without a match: run on the latest
    arrive(obj, 'joints', 65)
    assert obj.tick.call_count == 2         # nothing newer: no second mixed cycle
    clock[0] += 0.002; arrive(obj, 'odom', 70); arrive(obj, 'feet', 70); arrive(obj, 'joints', 70)
    assert obj.tick.call_count == 3         # a complete sample runs at once


def test_no_fallback_when_disabled(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(node.time, 'monotonic', lambda: clock[0])
    obj = fake(fallback=0.0)
    for name in ('odom', 'feet', 'joints'):
        arrive(obj, name, 1)
    for k in range(10):
        clock[0] += 0.02; arrive(obj, 'joints', 10 + 2 * k); arrive(obj, 'odom', 11 + 2 * k); arrive(obj, 'feet', 11 + 2 * k)
    assert obj.tick.call_count == 1
