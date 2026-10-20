"""Read-only telemetry invariants; no ROS participant or hardware is created."""
from sensor_msgs.msg import JointState
import pytest
from humanoid_safety.motor_web_node import MotorStatusCache


def message(names=('a', 'b'), stamp=100., positions=None):
    msg = JointState(name=list(names), position=positions or [0.1] * len(names))
    ns = int(stamp * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    return msg


def test_partial_traffic_does_not_renew_other_joint():
    cache = MotorStatusCache({'a': {}, 'b': {}}, timeout=.5)
    assert cache.observe(message(), 10, 100)
    assert cache.observe(message(('a',), 100.4), 10.4, 100.4)
    a, b = cache.snapshot(10.6, 100.6)['motors']
    assert a['online'] and not b['online']
    assert b['position_rad'] == .1
    assert b['velocity_radps'] is None and b['effort_nm'] is None


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_stale_future_and_unstamped_input_rejected(stamp):
    cache = MotorStatusCache()
    assert not cache.observe(message(stamp=stamp), 10, 100)
    assert cache.snapshot(10, 100)['motors'] == []


def test_reordered_samples_do_not_replace_newer_state():
    cache = MotorStatusCache()
    cache.observe(message(stamp=100.1), 10.1, 100.1)
    cache.observe(message(stamp=100., positions=[.9, .9]), 10.2, 100.2)
    assert cache.snapshot(10.2, 100.2)['motors'][0]['position_rad'] == .1


def test_bad_arrays_and_nan_never_reach_http_json():
    cache = MotorStatusCache()
    for msg in (message(positions=[.1]), message(positions=[float('nan'), .1]),
                message(names=('a', 'a'))):
        assert not cache.observe(msg, 10, 100)
    assert cache.snapshot(10, 100)['motors'] == []


def test_registry_missing_joint_remains_offline_and_temp_index_is_not_can_id():
    cache = MotorStatusCache({'a': {'motor_id': 42}, 'b': {'motor_id': 7}})
    cache.observe(message(('a',)), 10, 100)
    cache.observe_temperatures('{"0": {"temperature": 31}, "42": {"temperature": 99}}', 10)
    result = cache.snapshot(10.1, 100.1)
    assert result['estop'] is None
    assert result['motors'][0]['temperature_c'] == 31
    assert not result['motors'][1]['online']
    assert cache.snapshot(10.6, 100.6)['motors'][0]['temperature_c'] is None


def test_paused_ros_clock_does_not_keep_sample_online():
    cache = MotorStatusCache()
    cache.observe(message(), 10, 100)
    assert not cache.snapshot(11, 100)['connected']
