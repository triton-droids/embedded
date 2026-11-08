"""The external status process must render exactly what the inline path renders.

``status_publisher:=inline`` builds the DiagnosticArray in the supervisor process;
``status_publisher:=external`` sends the same snapshot as JSON and a separate
process renders it.  Consumers (the Web monitor, RViz tooling) must not be able
to tell which mode is active, so these tests pin the equivalence and the
fall-back behaviour of each mode.
"""
import json

import pytest
import rclpy
from std_msgs.msg import String

from humanoid_safety.diagnostics_node import SafetyDiagnosticsNode
from humanoid_safety.safety_node import SafetyNode
from humanoid_safety.status_messages import render

SNAPSHOT = {
    'schema_version': 1,
    'latched': True,
    'reason': 'imu_freshness: no IMU received within 0.250s',
    'imu_received': 12,
    'imu_rejected_source': 2,
    'imu_rejected_orientation': 1,
    'estop_publish_failures': 0,
    'checks': [
        {'name': 'fall_detection', 'active': False, 'healthy': True,
         'clearable': False, 'reason': ''},
        {'name': 'imu_freshness', 'active': True, 'healthy': False,
         'clearable': True, 'reason': 'stale'},
    ],
}


def _level(item):
    """``DiagnosticStatus.level`` is a ``byte`` field: bytes in Jazzy and Humble."""
    value = item.level
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


def _array_fields(array):
    """Comparable view of a DiagnosticArray: the JSON round-trip is lossy for bytes."""
    return [(item.name, item.level, item.message,
             [(kv.key, kv.value) for kv in item.values])
            for item in array.status]


def test_render_is_stable_across_the_json_round_trip():
    """The external payload travels as JSON: no diagnostic field may degrade."""
    direct = render(SNAPSHOT)
    through_wire = render(json.loads(json.dumps(SNAPSHOT)))
    assert _array_fields(direct) == _array_fields(through_wire)


def test_render_maps_supervisor_state_to_the_documented_levels():
    array = render(SNAPSHOT)
    assert array.status[0].name == 'humanoid_safety/supervisor'
    assert _level(array.status[0]) == 2  # ERROR while latched
    assert _level(array.status[1]) == 0  # healthy, inactive -> OK
    assert _level(array.status[2]) == 2  # active check -> ERROR
    assert dict((kv.key, kv.value) for kv in array.status[0].values)['latched'] == 'True'


@pytest.mark.parametrize('mode,expect_array', [('inline', True), ('external', False)])
def test_the_supervisor_creates_only_the_publisher_its_mode_needs(mode, expect_array):
    rclpy.init(args=['--ros-args', '-p', f'status_publisher:={mode}'])
    node = None
    try:
        node = SafetyNode()
        assert (node._status_pub is not None) is expect_array
        assert (node._state_pub is not None) is (not expect_array)
        assert node._publish_external is (not expect_array)
        # The snapshot is what an external renderer needs; it stays plain data.
        # ``results`` is only populated by an evaluation, as on the real node.
        node.supervisor.evaluate(node._now_seconds())
        snapshot = node._status_snapshot()
        assert snapshot['schema_version'] == 1
        assert {check['name'] for check in snapshot['checks']} >= {'imu_freshness'}
        json.dumps(snapshot)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_unknown_status_publisher_is_rejected():
    rclpy.init(args=['--ros-args', '-p', 'status_publisher:=sometimes'])
    try:
        with pytest.raises(ValueError):
            SafetyNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_diagnostics_process_renders_the_payload_it_receives():
    rclpy.init()
    node = None
    try:
        node = SafetyDiagnosticsNode()
        assert node._last_state is None
        node._on_state(String(data=json.dumps(SNAPSHOT)))
        assert node._last_state['latched'] is True
        assert node._published == 1
        # Malformed payloads are ignored instead of crashing the status path.
        node._on_state(String(data='{not json'))
        assert node._published == 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_missing_supervisor_is_reported_until_state_arrives():
    """A dead supervisor must not leave a stale 'all clear' on the Web monitor.

    The diagnostic repeats at the status rate rather than once, because DDS
    drops a one-shot to a subscriber that has not finished discovery yet.
    """
    import time

    rclpy.init(args=['--ros-args', '-p', 'stale_after_s:=0.0', '-p', 'status_rate_hz:=100.0'])
    node = None
    try:
        node = SafetyDiagnosticsNode()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and node._published < 2:
            time.sleep(0.02)
        assert node._published >= 2, 'stale supervisor was reported only once'
        assert node._reported_stale is True
        node._on_state(String(data=json.dumps(SNAPSHOT)))
        assert node._reported_stale is False
        assert node._last_state['latched'] is True
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
