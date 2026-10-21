"""Read-only telemetry invariants; no ROS participant or hardware is created."""
import json
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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


def telemetry(records):
    return json.dumps(dict(schema_version=1, motors=records))


def record(name='a', motor_id=42, stamp=100., temperature=31., interface='can0'):
    return dict(joint_name=name, can_interface=interface, motor_id=motor_id,
                model='rs-03', source_stamp=stamp, temperature_c=temperature)


def registry():
    return {'a': dict(can_interface='can0', motor_id=42, model='rs-03'),
            'b': dict(can_interface='can1', motor_id=7, model='rs-03')}


def diagnostic(stamp=100., level=0):
    msg = DiagnosticArray(status=[DiagnosticStatus(
        name='humanoid_safety/supervisor', hardware_id='humanoid', level=bytes([level]),
        message='test health')])
    ns = int(stamp * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    return msg


def test_reordered_registry_and_records_keep_motor_identity():
    cache = MotorStatusCache(dict(reversed(list(registry().items()))))
    cache.observe(message(), 10, 100)
    cache.observe_temperatures(telemetry([record(), record('b', 7, temperature=45,
                                                        interface='can1')]), 10, 100)
    motors = {m['joint_name']: m for m in cache.snapshot(10.1, 100.1)['motors']}
    assert motors['a']['temperature_c'] == 31
    assert motors['b']['temperature_c'] == 45
    assert motors['b']['can_interface'] == 'can1'


@pytest.mark.parametrize('changes', [dict(motor_id=99), dict(can_interface='can9'),
                                    dict(model='other')])
def test_mismatched_registry_identity_is_unknown(changes):
    cache = MotorStatusCache(registry())
    cache.observe(message(), 10, 100)
    item = record(); item.update(changes)
    cache.observe_temperatures(telemetry([item]), 10, 100)
    assert cache.snapshot(10, 100)['motors'][0]['temperature_c'] is None


def test_legacy_indices_and_missing_temperature_are_unknown():
    cache = MotorStatusCache(registry())
    cache.observe(message(), 10, 100)
    assert not cache.observe_temperatures('{"0":{"temperature":31}}', 10, 100)
    item = record(); del item['temperature_c']
    cache.observe_temperatures(telemetry([item]), 10, 100)
    assert all(m['temperature_c'] is None for m in cache.snapshot(10, 100)['motors'])


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_temperature_source_age_is_checked(stamp):
    cache = MotorStatusCache(registry())
    cache.observe(message(), 10, 100)
    cache.observe_temperatures(telemetry([record(stamp=stamp)]), 10, 100)
    assert cache.snapshot(10, 100)['motors'][0]['temperature_c'] is None


def test_replay_cannot_renew_temperature_receipt_time():
    cache = MotorStatusCache(registry())
    cache.observe_temperatures(telemetry([record()]), 10, 100)
    cache.observe_temperatures(telemetry([record(temperature=90)]), 10.4, 100.4)
    cache.observe(message(stamp=100.6), 10.6, 100.6)
    assert cache.snapshot(10.6, 100.6)['motors'][0]['temperature_c'] is None
    assert cache.temperatures['a'][2] == 31


def test_supervisor_death_preserves_last_estop_but_not_current_health():
    cache = MotorStatusCache()
    cache.observe_estop(False, 10)
    assert cache.snapshot(10, 100)['estop'] is None
    assert cache.observe_supervisor(diagnostic(), 10, 100)
    assert cache.snapshot(10, 100)['estop'] is False
    stale = cache.snapshot(10010, 10100)
    assert stale['estop'] is None and stale['last_reported_estop'] is False
    assert not stale['supervisor_online'] and stale['supervisor_level'] is None


def test_replayed_supervisor_and_bool_do_not_renew_health():
    cache = MotorStatusCache()
    cache.observe_supervisor(diagnostic(), 10, 100)
    assert not cache.observe_supervisor(diagnostic(), 10.4, 100.4)
    cache.observe_estop(False, 10.6)
    assert not cache.snapshot(10.6, 100.6)['supervisor_online']
    assert cache.snapshot(10.6, 100.6)['estop'] is None


def test_paused_clock_and_diagnostic_warning_are_separate_from_liveness():
    cache = MotorStatusCache()
    cache.observe_estop(True, 10)
    cache.observe_supervisor(diagnostic(level=1), 10, 100)
    live = cache.snapshot(10, 100)
    assert live['supervisor_online'] and live['supervisor_level'] == 1
    assert live['estop'] is True
    assert not cache.snapshot(11, 100)['supervisor_online']


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_invalid_supervisor_source_stamp_does_not_renew_health(stamp):
    cache = MotorStatusCache()
    assert not cache.observe_supervisor(diagnostic(stamp), 10, 100)


def test_paused_ros_clock_does_not_keep_sample_online():
    cache = MotorStatusCache()
    cache.observe(message(), 10, 100)
    assert not cache.snapshot(11, 100)['connected']


def test_unregistered_monitor_uses_publisher_identity():
    cache = MotorStatusCache()
    cache.observe(message(('a',)), 10, 100)
    cache.observe_temperatures(telemetry([record()]), 10, 100)
    motor = cache.snapshot(10, 100)['motors'][0]
    assert (motor['motor_id'], motor['can_interface'], motor['model']) == (42, 'can0', 'rs-03')
    assert motor['temperature_c'] == 31


@pytest.mark.parametrize('payload', ['[]', '{}', '{broken',
    '{"schema_version":2,"motors":[]}',
    telemetry([record(), record()])])
def test_malformed_or_unsupported_telemetry_is_rejected(payload):
    cache = MotorStatusCache(registry())
    assert not cache.observe_temperatures(payload, 10, 100)
    assert not cache.temperatures


def test_diagnostics_for_another_component_are_not_supervisor_heartbeat():
    cache = MotorStatusCache()
    msg = diagnostic()
    msg.status[0].name = 'humanoid_safety/imu'
    assert not cache.observe_supervisor(msg, 10, 100)


def test_config_hot_reload_keeps_all_motors_and_clears_changed_identity(tmp_path):
    from humanoid_safety.motor_web_node import LiveMotorConfig
    path = tmp_path / 'motors.yaml'
    def write(motors):
        import yaml
        path.write_text(yaml.safe_dump({'motor_control_node': {'ros__parameters': {
            'default_can_interface': 'can1', 'hardware_verified': False, 'motors': motors}}}))
    cfg = {'a': {'motor_id': 1, 'model': 'rs-03', 'kp': 20},
           'b': {'motor_id': 2, 'model': 'rs-04', 'kp': 30}}
    write(cfg)
    cache = MotorStatusCache()
    live = LiveMotorConfig(path, cache)
    live.refresh(10)
    assert live.error is None
    assert [m['joint_name'] for m in cache.snapshot(10, 100)['motors']] == ['a', 'b']
    assert all(not m['online'] for m in cache.snapshot(10, 100)['motors'])
    cache.observe(message(('a',), positions=[.4]), 10, 100)
    cfg['a']['motor_id'] = 3
    cfg['c'] = cfg.pop('b')
    write(cfg)
    live.refresh(10.2)  # Polling does not repeatedly parse YAML.
    assert cache.registry['a']['motor_id'] == 1
    live.refresh(11.1)
    motors = cache.snapshot(11.1, 100)['motors']
    assert [m['joint_name'] for m in motors] == ['a', 'c']
    assert motors[0]['motor_id'] == 3 and motors[0]['position_rad'] is None
    assert motors[0]['can_interface'] == 'can1'
    assert live.snapshot()['parameters']['hardware_verified'] is False


def test_invalid_config_retains_last_valid_and_recovers(tmp_path):
    from humanoid_safety.motor_web_node import LiveMotorConfig
    path = tmp_path / 'motors.yaml'
    valid = 'motor_control_node:\n  ros__parameters:\n    motors:\n      a: {motor_id: 1, model: rs-03}\n'
    path.write_text(valid)
    cache = MotorStatusCache()
    live = LiveMotorConfig(path, cache)
    live.refresh(10)
    for i, invalid in enumerate(['motor_control_node: [', 'null',
            valid.replace('motor_id: 1', 'motor_id: true'), valid + '    KP: .nan\n']):
        path.write_text(invalid)
        live.refresh(12 + i * 2)
        assert live.error
        assert list(cache.registry) == ['a']
        assert live.parameters['motors']['a']['motor_id'] == 1
    path.unlink()
    live.refresh(22)
    assert live.error and list(cache.registry) == ['a']
    path.write_text(valid)
    live.refresh(24)
    assert live.error is None
