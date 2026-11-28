"""Exercise production callbacks with deterministic clocks and a mock CAN bus.

No ROS node, serial port, CAN socket, model or physical motor is started.
"""
import queue
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from motor_control_interfaces.msg import MotorCommand
from sensor_msgs.msg import JointState

from motor_control_hybrid.python_can_node import PythonCanNode
from motor_control_hybrid.policy_bridge_node import PolicyBridgeNode
from motor_control_hybrid.can_transport import LatestCommandMailbox


def ros_clock(seconds):
    return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(seconds * 1e9)))


def stamp(message, seconds=100.0):
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = int((seconds % 1) * 1e9)
    return message


@pytest.fixture
def can_node():
    node = PythonCanNode.__new__(PythonCanNode)
    node.telemetry_stamps = {}
    node.raw_feedback = {}
    node.state_lock = threading.Lock()
    node.can_rt_enabled = False
    node.can_tx_rt_priority = 20
    node.can_rx_rt_priority = 21
    bus = Mock(channel='can1')
    node.bus_by_joint = {'a': bus, 'b': bus}
    node.buses = {'can1': bus}
    node.motor_name_by_joint = {'a': 'motor_1', 'b': 'motor_2'}
    node.bus_locks = {'can1': threading.Lock()}
    node.current_mode_by_joint = {'a': None, 'b': None}
    node.motor_settings = {n: SimpleNamespace(kp=10, kd=1) for n in ('a', 'b')}
    node.command_queue = LatestCommandMailbox()
    node.active_commands = {}
    node.active_command_lock = threading.Lock()
    node.command_timeout_s = 0.5
    node.command_deadlines = {}
    node.inhibited_since = {'a': float('-inf'), 'b': float('-inf')}
    node.pending_disable = set()
    node.enable_pending = {}
    node.submitted_state = {}
    node.enable_reports = {}
    node.enable_report_lock = threading.Lock()
    node._awaiting_enable_ack = threading.Event()
    node.estop_active = threading.Event()
    node.transport_fault = threading.Event()
    node._tx_wakeup = threading.Event()
    node.timing = {'tx_errors': 0}
    node.get_clock = lambda: ros_clock(100)
    node.logger = Mock()
    node.get_logger = lambda: node.logger
    node._set_mode_raw = Mock()
    node._try_read_feedback = Mock()
    node.firmware_pending, node.param_readback, node.enabled_at = {}, {}, {}
    return node


def command(node, mode, names=('a',), source_time=100):
    node._cmd_callback(stamp(MotorCommand(joint_name=list(names), mode=[mode]), source_time))
    node._drain_command_queue()
    if mode == MotorCommand.MODE_ENABLE:
        node.enable_reports.update({joint: (time.monotonic(), 2) for joint in names
                                    if joint in node.enable_pending})
        node._process_enable_reports()


def test_can_timeout_disables_once_and_requires_new_enable(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        command(node, MotorCommand.MODE_ENABLE)
        command(node, MotorCommand.MODE_MOTION)
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.6):
        node._expire_commands()
        node._expire_commands()
        assert not node.active_commands
        node.bus_by_joint['a'].disable.assert_called_once_with('motor_1')
        command(node, MotorCommand.MODE_MOTION)
        assert not node.active_commands
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.7), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        command(node, MotorCommand.MODE_ENABLE)
        command(node, MotorCommand.MODE_MOTION)
        assert 'a' in node.active_commands


def test_repeated_disable_is_sent_every_time_but_logged_at_info_once(can_node):
    node = can_node
    bus = node.bus_by_joint['a']

    def info_lines():
        return [c.args[0] for c in node.logger.info.call_args_list]

    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        for _ in range(50):  # one second of idle controller output
            command(node, MotorCommand.MODE_DISABLE)
        assert bus.disable.call_count == 50
        assert info_lines() == ['Disable submitted for a']
        assert node.logger.debug.call_count == 49
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=11), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        command(node, MotorCommand.MODE_ENABLE)
        command(node, MotorCommand.MODE_MOTION)
        command(node, MotorCommand.MODE_DISABLE)
        command(node, MotorCommand.MODE_DISABLE)
    assert info_lines() == ['Disable submitted for a', 'Enable submitted for a',
                            'Disable submitted for a']
    assert bus.disable.call_count == 52


def test_other_joint_commands_do_not_refresh_expired_joint(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        command(node, MotorCommand.MODE_ENABLE, ('a', 'b'))
        command(node, MotorCommand.MODE_MOTION, ('a', 'b'))
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.4):
        command(node, MotorCommand.MODE_MOTION, ('b',))
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.6):
        node._expire_commands()
    assert set(node.active_commands) == {'b'}
    node.bus_by_joint['a'].disable.assert_called_once_with('motor_1')


def test_queued_enable_and_motion_expire_before_execution(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        for mode in (MotorCommand.MODE_ENABLE, MotorCommand.MODE_MOTION):
            node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[mode])))
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=11):
        node._drain_command_queue()
    node.bus_by_joint['a'].enable.assert_not_called()
    assert not node.active_commands


def test_failed_timeout_disable_is_retried(can_node):
    node = can_node
    node.command_deadlines['a'] = 9
    node.bus_by_joint['a'].disable.side_effect = [RuntimeError('offline'), None]
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        node._expire_commands()
        assert node.pending_disable == {'a'}
        node._expire_commands()
    assert not node.pending_disable
    assert node.bus_by_joint['a'].disable.call_count == 2


def test_failed_enable_acknowledgement_is_followed_by_disable(can_node):
    node = can_node
    node.bus_by_joint['a'].enable.side_effect = RuntimeError('ack missing')
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        command(node, MotorCommand.MODE_ENABLE)
    node.bus_by_joint['a'].disable.assert_called_once_with('motor_1')
    assert 'a' in node.inhibited_since
    assert not node.command_deadlines


@pytest.mark.parametrize('source_time', [0, 99, 101])
def test_can_rejects_unstamped_stale_and_future_motion(can_node, source_time):
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        can_node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[2]), source_time))
    assert can_node.command_queue.empty()


def test_can_checks_deadline_again_at_actual_send(can_node):
    node = can_node
    node.command_deadlines['a'] = 10.5
    node.inhibited_since.clear()
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.6):
        node._send_active_command({'joint': 'a', 'mode': MotorCommand.MODE_MOTION})
    node.bus_by_joint['a'].write_operation_frame.assert_not_called()
    node.bus_by_joint['a'].disable.assert_called_once()


def test_source_age_consumes_command_lifetime_and_disable_needs_no_stamp(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10), \
            patch('motor_control_hybrid.python_can_node.time.sleep'):
        command(node, MotorCommand.MODE_ENABLE, source_time=99.75)
        assert node.command_deadlines['a'] == pytest.approx(10.25)
        command(node, MotorCommand.MODE_DISABLE, source_time=0)
    node.bus_by_joint['a'].disable.assert_called_once()
    assert 'a' in node.inhibited_since


def test_enable_queued_before_timeout_cannot_rearm_joint(can_node):
    node = can_node
    node.command_deadlines['a'] = 10
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=9.9):
        node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[3])))
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.1):
        node._drain_command_queue()
    node.bus_by_joint['a'].enable.assert_not_called()


def test_published_feedback_stamp_preserves_oldest_receipt_age(can_node):
    node = can_node
    node.state_lock = threading.Lock()
    node.state_buffer = {'a': (1., 0., 0., 25., 9.95), 'b': (2., 0., 0., 25., 10.)}
    node.feedback_timeout_s = 0.1
    node.joint_state_pub = Mock()
    node._publish_motor_status = Mock()
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=100)))
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        node._publish_states()
    msg = node.joint_state_pub.publish.call_args.args[0]
    assert msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 == pytest.approx(99.95)


def test_blocked_rx_cannot_block_actual_tx_worker(can_node, monkeypatch):
    """Hold RX inside recv while the production TX loop sends a target."""
    node = can_node
    rx_entered, release_rx, target_sent = threading.Event(), threading.Event(), threading.Event()
    bus = node.bus_by_joint['a']
    bus.channel_handler.recv.side_effect = lambda **kwargs: (
        rx_entered.set(), release_rx.wait(1.0), None)[-1]
    bus.write_operation_frame.side_effect = lambda *args: target_sent.set()
    node.motor_settings['a'].command = lambda value, measured=None: value
    node.running = True
    node.tx_period_s = 0.02
    node.feedback_timeout_s = 0.1
    node.default_kp, node.default_kd, node.max_vel_rad_s = 10, 1, 4.5
    node.timing.update(tx_cycles=0, tx_max_work_ms=0)
    node.inhibited_since.clear()
    node.current_mode_by_joint['a'] = 0
    target = dict(joint='a', mode=2, position=0.2, velocity=0, torque=0, kp=10, kd=1)
    node.active_commands['a'] = target
    node.command_deadlines['a'] = time.monotonic() + 1.0
    monkeypatch.setattr('motor_control_hybrid.python_can_node.rclpy.ok', lambda: True)
    rx = threading.Thread(target=node._can_rx_loop, args=(bus,))
    tx = threading.Thread(target=node._can_tx_loop)
    rx.start()
    try:
        assert rx_entered.wait(1)
        tx.start()
        assert target_sent.wait(1), 'TX stalled behind RX'
        assert not release_rx.is_set()
    finally:
        node.running = False
        node._tx_wakeup.set()
        release_rx.set()
        rx.join(1)
        if tx.ident is not None:
            tx.join(1)
    assert not rx.is_alive() and not tx.is_alive()


def test_tx_waits_only_period_remainder(can_node, monkeypatch):
    node = can_node
    node.running = True
    node.tx_period_s = 0.02
    node._expire_commands = Mock()
    node._drain_command_queue = Mock()
    node.timing.update(tx_cycles=0, tx_max_work_ms=0)
    clock = iter([10.0, 10.006])
    monkeypatch.setattr('motor_control_hybrid.python_can_node.time.monotonic', lambda: next(clock))
    monkeypatch.setattr('motor_control_hybrid.python_can_node.rclpy.ok', lambda: True)
    waits = []

    def wait(seconds):
        waits.append(seconds)
        node.running = False

    node._tx_wakeup = SimpleNamespace(clear=lambda: None, wait=wait)
    node._can_tx_loop()
    assert waits == pytest.approx([0.014])


def test_transport_fault_cannot_be_reset_by_supervisor_false(can_node):
    node = can_node
    node.transport_fault.set()
    node._safety_estop_callback(SimpleNamespace(data=False))
    node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[3])))
    assert node.command_queue.empty()


def test_enable_waits_asynchronously_for_fresh_run_status(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[3])))
        node._drain_command_queue()
        assert 'a' in node.inhibited_since
        assert 'a' in node.enable_pending
        command(node, MotorCommand.MODE_MOTION)
        node._send_active_command(node.active_commands['a'])
        node.bus_by_joint['a'].write_operation_frame.assert_not_called()
        # Reset mode, or a pre-enable run sample, must not unlock output.
        for report in ((10, 0), (9.9, 2)):
            node.enable_reports['a'] = report
            node._process_enable_reports()
            assert 'a' in node.inhibited_since
        node.enable_reports['a'] = (10, 2)
        node._process_enable_reports()
        assert 'a' not in node.inhibited_since
        assert not node.enable_pending


def test_motion_traffic_cannot_keep_unacknowledged_enable_alive(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[3])))
        node._drain_command_queue()
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.4):
        command(node, MotorCommand.MODE_MOTION)
        assert node.command_deadlines['a'] == pytest.approx(10.5)
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.6):
        node._expire_commands()
    assert 'a' in node.inhibited_since
    assert not node.active_commands
    node.bus_by_joint['a'].disable.assert_called_once()


def test_virtual_can_round_trip_enable_ack_and_both_joint_targets(can_node, monkeypatch):
    """Run both real worker loops over an in-memory python-can interface."""
    import struct
    import uuid
    import can
    from robstride_dynamics import Motor
    from motor_control_hybrid.can_transport import NonblockingRobstrideBus
    from motor_control_hybrid.motor_configuration import MotorSettings

    node = can_node
    transport = NonblockingRobstrideBus('virtual', {
        'motor_1': Motor(1, 'rs-04'), 'motor_2': Motor(2, 'rs-02')}, {})
    transport._motor_by_id = {1: 'motor_1', 2: 'motor_2'}
    channel = f'watchdog-{uuid.uuid4()}'
    monkeypatch.setattr('motor_control_hybrid.python_can_node.rclpy.ok', lambda: True)
    with can.ThreadSafeBus(interface='virtual', channel=channel) as local, \
            can.Bus(interface='virtual', channel=channel) as peer:
        transport.channel_handler = local
        node.bus_by_joint = {'a': transport, 'b': transport}
        node.bus_locks = {'virtual': threading.Lock()}
        node.motor_settings = {joint: MotorSettings.from_config({}, {}) for joint in ('a', 'b')}
        node._joint_by_motor = {('virtual', 'motor_1'): 'a', ('virtual', 'motor_2'): 'b'}
        node.state_buffer = {}
        node.state_lock = threading.Lock()
        node.feedback_timeout_s = 0.1
        node.command_timeout_s = 2.0
        node.default_kp, node.default_kd, node.max_vel_rad_s = 10, 0.2, 4.5
        node.tx_period_s = 0.02
        node.timing.update(tx_cycles=0, tx_max_work_ms=0, rx_frames=0, rx_ignored=0)
        node._estop_disable_pending = threading.Event()
        node.running = True
        node._cmd_callback(stamp(MotorCommand(joint_name=['a', 'b'], mode=[3])))
        node._cmd_callback(stamp(MotorCommand(joint_name=['a', 'b'], mode=[2], position=[0.1, 0.2])))
        rx = threading.Thread(target=node._can_rx_loop, args=(transport,))
        tx = threading.Thread(target=node._can_tx_loop)
        rx.start()
        tx.start()
        seen = set()
        try:
            deadline = time.monotonic() + 2
            while len(seen) < 2 and time.monotonic() < deadline:
                frame = peer.recv(timeout=0.1)
                if frame is None:
                    continue
                kind = (frame.arbitration_id >> 24) & 31
                device = frame.arbitration_id & 255
                if kind == 3:
                    peer.send(can.Message(
                        arbitration_id=(2 << 24) | (((2 << 14) | device) << 8) | 255,
                        is_extended_id=True, data=struct.pack('>HHHH', 32767, 32767, 32767, 250)))
                elif kind == 1:
                    seen.add(device)
            assert seen == {1, 2}
            assert set(node.state_buffer) == {'a', 'b'}
            assert set(node.raw_feedback) == {'a', 'b'}
            assert all(sample[0].mode == 2 for sample in node.raw_feedback.values())
            assert not node.transport_fault.is_set()
        finally:
            node.running = False
            node._tx_wakeup.set()
            tx.join(1)
            rx.join(1)
            transport.channel_handler = None
        assert not tx.is_alive() and not rx.is_alive()


@pytest.fixture
def bridge():
    node = PolicyBridgeNode.__new__(PolicyBridgeNode)
    node.obs_builder = SimpleNamespace(real_joint_order=['a', 'b'])
    node.joint_pos_by_name = {}
    node.joint_vel_by_name = {}
    node.joint_sample_times = {}
    node.ang_vel_body = np.zeros(3)
    node.up_body = np.array([0, 0, 1])
    node.last_imu_time = None
    node.get_clock = lambda: ros_clock(100)
    node.get_parameter = lambda name: SimpleNamespace(value=0.25)
    node._warn_waiting = Mock()
    return node


def feedback(names=('a', 'b'), source_time=100):
    return stamp(JointState(name=list(names), position=[1.] * len(names),
                           velocity=[0.] * len(names)), source_time)


def test_bridge_partial_updates_do_not_refresh_missing_joint(bridge):
    with patch('motor_control_hybrid.policy_bridge_node.time.monotonic', return_value=10):
        bridge._joint_state_callback(feedback())
        assert bridge._inputs_ready()
    with patch('motor_control_hybrid.policy_bridge_node.time.monotonic', return_value=11):
        bridge._joint_state_callback(feedback(('a',)))
        assert not bridge._inputs_ready()


@pytest.mark.parametrize('source_time', [0, 99, 101])
def test_bridge_rejects_invalid_source_age(bridge, source_time):
    with patch('motor_control_hybrid.policy_bridge_node.time.monotonic', return_value=10):
        bridge._joint_state_callback(feedback(source_time=source_time))
        assert not bridge._inputs_ready()


@pytest.mark.parametrize('bad', ['velocity', 'nan', 'duplicate'])
def test_bridge_malformed_update_cannot_reuse_previous_state(bridge, bad):
    with patch('motor_control_hybrid.policy_bridge_node.time.monotonic', return_value=10):
        bridge._joint_state_callback(feedback())
        msg = feedback()
        if bad == 'velocity':
            msg.velocity = []
        elif bad == 'nan':
            msg.position[0] = float('nan')
        else:
            msg.name = ['a', 'a']
        bridge._joint_state_callback(msg)
        assert not bridge._inputs_ready()


def test_explicit_policy_deadline_is_not_renewed_by_can_receipt(can_node):
    node = can_node
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10):
        command(node, MotorCommand.MODE_ENABLE)
        msg = stamp(MotorCommand(joint_name=['a'], mode=[MotorCommand.MODE_MOTION]))
        msg.valid_until.sec, msg.valid_until.nanosec = 100, 25_000_000
        node._cmd_callback(msg)
        node._drain_command_queue()
        assert node.command_deadlines['a'] == pytest.approx(10.025)
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.026):
        node._expire_commands()
    assert 'a' not in node.active_commands
    assert 'a' in node.inhibited_since


def test_expired_explicit_deadline_rejects_even_recent_source(can_node):
    msg = stamp(MotorCommand(joint_name=['a'], mode=[MotorCommand.MODE_MOTION]), 99.99)
    msg.valid_until.sec, msg.valid_until.nanosec = 99, 999_000_000
    can_node._cmd_callback(msg)
    assert can_node.command_queue.empty()


@pytest.mark.parametrize('role,priority', [('TX', 20), ('RX can1', 21)])
def test_can_rt_targets_calling_thread(can_node, role, priority):
    can_node.can_rt_enabled = True
    with patch('motor_control_hybrid.python_can_node.os.sched_setscheduler') as setter:
        can_node._configure_can_scheduling(role, priority)
    assert setter.call_args.args[0] == 0
    assert setter.call_args.args[2].sched_priority == priority


def test_can_rt_permission_failure_keeps_worker_alive(can_node):
    can_node.can_rt_enabled = True
    logger = Mock()
    can_node.get_logger = lambda: logger
    with patch('motor_control_hybrid.python_can_node.os.sched_setscheduler',
               side_effect=PermissionError('denied')):
        can_node._configure_can_scheduling('TX', 20)
    logger.warning.assert_called_once()


def test_named_telemetry_keeps_feedback_stamp_and_legacy_topic(can_node):
    import json
    node = can_node
    node.state_lock = threading.Lock()
    node.state_buffer = {'a': (.1, .2, .3, 31., time.monotonic())}
    node.telemetry_stamps = {'a': 99.98}
    node.motor_identity = {'a': dict(joint_name='a', can_interface='can1',
                                    motor_id=42, model='rs-03')}
    node.motor_index_map = {'a': 0}
    node.feedback_timeout_s = .5
    node.motor_status_pub, node.telemetry_pub = Mock(), Mock()
    node._publish_motor_status()
    payload = json.loads(node.telemetry_pub.publish.call_args.args[0].data)
    assert payload['schema_version'] == 1
    assert payload['motors'][0] == dict(node.motor_identity['a'],
                                       source_stamp=99.98, temperature_c=31., fault_bits=0)
    assert json.loads(node.motor_status_pub.publish.call_args.args[0].data)['0']['temperature'] == 31.
    node._publish_motor_status()
    assert json.loads(node.telemetry_pub.publish.call_args.args[0].data)['motors'][0]['source_stamp'] == 99.98


def test_telemetry_source_stamp_preserves_received_feedback_age(can_node):
    node = can_node
    node.state_lock = threading.Lock()
    node.state_buffer = {}
    node.motor_settings['a'].feedback = lambda *values: values
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.):
        node._update_state('a', .1, .2, .3, 31., received_at=9.98)
    assert node.telemetry_stamps['a'] == pytest.approx(99.98)
    assert node.state_buffer['a'][4] == 9.98


def _feed_rx(node, monkeypatch, positions):
    """Run the production RX loop over raw positions received 2 ms apart."""
    from motor_control_hybrid.can_transport import MotorFeedback
    from motor_control_hybrid.motor_configuration import MotorSettings
    node.motor_settings = {n: MotorSettings.from_config({'model': 'rs-03'}, {}) for n in ('a', 'b')}
    node.feedback_timeout_s = 0.1
    node.state_buffer = {}
    node._joint_by_motor = {('can1', 'motor_1'): 'a'}
    node._estop_disable_pending = threading.Event()
    node.timing.update(rx_frames=0, rx_ignored=0)
    bus = node.buses['can1']
    now = time.time()
    samples = iter([MotorFeedback('motor_1', p, 0.0, 0.0, 30.0, 2, 0, now - 0.01 + 0.002 * i)
                    for i, p in enumerate(positions)])

    def recv(timeout):
        try:
            return next(samples)
        except StopIteration:
            node.running = False
            return None
    bus.channel_handler.recv.side_effect = recv
    bus.decode_feedback.side_effect = lambda frame: frame
    node.running = True
    monkeypatch.setattr('motor_control_hybrid.python_can_node.rclpy.ok', lambda: True)
    monkeypatch.setattr('motor_control_hybrid.python_can_node.time.sleep', lambda s: None)
    node._can_rx_loop(bus)


def test_rx_latches_fault_on_coordinate_jump(can_node, monkeypatch):
    # 0.8 rad in 2 ms (an external mechanical-zero write) is impossible motion.
    _feed_rx(can_node, monkeypatch, [0.8, 0.8, 0.0])
    assert can_node.transport_fault.is_set() and can_node._estop_disable_pending.is_set()
    assert 'jumped' in can_node.logger.error.call_args.args[0]


def test_rx_accepts_physical_motion(can_node, monkeypatch):
    _feed_rx(can_node, monkeypatch, [0.0, 0.03, 0.06, 0.09])  # 15 rad/s
    assert not can_node.transport_fault.is_set(), can_node.logger.error.call_args
    assert can_node.state_buffer['a'][0] == pytest.approx(0.09)


def _verified_enable_node(can_node):
    from motor_control_hybrid.motor_configuration import MotorSettings
    node = can_node
    node.motor_settings = {n: MotorSettings.from_config(
        {'model': 'rs-03', 'firmware_can_timeout_s': 1.0}, {}) for n in ('a', 'b')}
    node._write_parameter = Mock()
    return node


def test_enable_is_deferred_until_can_timeout_reads_back(can_node):
    node = _verified_enable_node(can_node)
    bus = node.bus_by_joint['a']
    command(node, MotorCommand.MODE_ENABLE)  # includes a stale Motor-mode report
    bus.enable.assert_not_called()
    assert 'a' in node.enable_pending
    bus.read_parameter.assert_called_with('motor_1', 0x7028)
    # A wrong value never enables the motor.
    node.param_readback[('a', 0x7028)] = (bytes.fromhex('00000000'), time.monotonic())
    node._process_firmware_readback()
    bus.enable.assert_not_called()
    node.param_readback[('a', 0x7028)] = ((20000).to_bytes(4, 'little'), time.monotonic())
    node._process_firmware_readback()
    bus.enable.assert_called_once_with('motor_1')
    assert 'a' not in node.firmware_pending and 'a' in node.enable_pending


def test_motor_leaving_run_mode_latches_fault(can_node):
    from motor_control_hybrid.motor_configuration import MotorSettings
    node = can_node
    node.motor_settings = {n: MotorSettings.from_config({'model': 'rs-03'}, {}) for n in ('a', 'b')}
    node.feedback_timeout_s = 0.1
    node._estop_disable_pending = threading.Event()
    node.inhibited_since.clear()
    node.enabled_at['a'] = time.monotonic() - 0.2
    node.current_mode_by_joint['a'] = 0
    # Fresh status after the enable shows reset mode (0): the motor watchdog fired.
    node.raw_feedback['a'] = (SimpleNamespace(position=0.0, mode=0), time.monotonic(), 0.0)
    node._send_active_command(dict(joint='a', mode=2, position=0.0, velocity=0.0,
                                   acceleration=0.0, torque=0.0, kp=10, kd=1))
    assert node.transport_fault.is_set() and node._estop_disable_pending.is_set()
    node.bus_by_joint['a'].write_operation_frame.assert_not_called()



def _run_virtual_motor(can_node, monkeypatch, honour_writes):
    """Worker loops against a simulated motor that stores type-18 writes.

    Returns the frame kinds per device in arrival order and whether an ENABLE
    reached the motor before its canTimeout held the written value.
    """
    import struct
    import uuid
    import can
    from robstride_dynamics import Motor
    from motor_control_hybrid.can_transport import NonblockingRobstrideBus
    from motor_control_hybrid.motor_configuration import MotorSettings

    node = can_node
    transport = NonblockingRobstrideBus('virtual', {'motor_1': Motor(1, 'rs-04')}, {})
    transport._motor_by_id = {1: 'motor_1'}
    channel = f'firmware-{uuid.uuid4()}'
    monkeypatch.setattr('motor_control_hybrid.python_can_node.rclpy.ok', lambda: True)
    memory, kinds, unsafe_enable = {}, [], []
    with can.ThreadSafeBus(interface='virtual', channel=channel) as local, \
            can.Bus(interface='virtual', channel=channel) as peer:
        transport.channel_handler = local
        node.bus_by_joint = {'a': transport}
        node.motor_name_by_joint = {'a': 'motor_1'}
        node.current_mode_by_joint = {'a': None}
        node.inhibited_since = {'a': float('-inf')}
        node.bus_locks = {'virtual': threading.Lock()}
        node.motor_settings = {'a': MotorSettings.from_config(
            {'model': 'rs-04', 'firmware_can_timeout_s': 1.0}, {})}
        node._joint_by_motor = {('virtual', 'motor_1'): 'a'}
        node.state_buffer = {}
        node.feedback_timeout_s = 0.1
        node.command_timeout_s = 0.8
        node.default_kp, node.default_kd, node.max_vel_rad_s = 10, 0.2, 4.5
        node.tx_period_s = 0.02
        node.timing.update(tx_cycles=0, tx_max_work_ms=0, rx_frames=0, rx_ignored=0)
        node._estop_disable_pending = threading.Event()
        node.running = True
        node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[3])))
        rx = threading.Thread(target=node._can_rx_loop, args=(transport,))
        tx = threading.Thread(target=node._can_tx_loop)
        rx.start()
        tx.start()
        try:
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                frame = peer.recv(timeout=0.05)
                if frame is None:
                    continue
                kind = (frame.arbitration_id >> 24) & 31
                device, host = frame.arbitration_id & 255, (frame.arbitration_id >> 8) & 255
                kinds.append(kind)
                index = struct.unpack('<H', bytes(frame.data[:2]))[0]
                if kind == 18 and honour_writes:
                    memory[index] = bytes(frame.data[4:8])
                elif kind == 17:
                    peer.send(can.Message(
                        arbitration_id=(17 << 24) | (device << 8) | host, is_extended_id=True,
                        data=bytes(frame.data[:4]) + memory.get(index, bytes(4))))
                elif kind == 3:
                    if memory.get(0x7028) != (20000).to_bytes(4, 'little'):
                        unsafe_enable.append(True)
                    peer.send(can.Message(
                        arbitration_id=(2 << 24) | (((2 << 14) | device) << 8) | host,
                        is_extended_id=True, data=struct.pack('>HHHH', 32767, 32767, 32767, 250)))
        finally:
            node.running = False
            node._tx_wakeup.set()
            tx.join(1)
            rx.join(1)
    return kinds, bool(unsafe_enable), node


def test_virtual_motor_enable_after_verified_can_timeout(can_node, monkeypatch):
    kinds, unsafe, node = _run_virtual_motor(can_node, monkeypatch, honour_writes=True)
    assert 3 in kinds and not unsafe
    assert kinds.index(18) < kinds.index(17) < kinds.index(3)
    assert 'a' not in node.enable_pending  # Motor-mode ack completed the enable


def test_virtual_motor_that_ignores_writes_is_never_enabled(can_node, monkeypatch):
    kinds, unsafe, node = _run_virtual_motor(can_node, monkeypatch, honour_writes=False)
    assert 3 not in kinds and not unsafe
    assert kinds.count(17) >= 2  # readback retried


def test_enable_ack_arriving_during_the_enable_send_is_accepted(can_node):
    """A motor answers ENABLE before bus.enable returns; that ack must count."""
    node = _verified_enable_node(can_node)
    bus = node.bus_by_joint['a']
    bus.enable.side_effect = lambda motor: node.enable_reports.update(a=(time.monotonic(), 2))
    node._cmd_callback(stamp(MotorCommand(joint_name=['a'], mode=[MotorCommand.MODE_ENABLE])))
    node._drain_command_queue()
    node.param_readback[('a', 0x7028)] = ((20000).to_bytes(4, 'little'), time.monotonic())
    node._process_firmware_readback()
    node._process_enable_reports()
    assert 'a' not in node.enable_pending and 'a' not in node.inhibited_since
