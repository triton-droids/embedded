"""Nonblocking TX, strict RX routing and bounded latest-command exchange."""
import queue
import struct
from unittest.mock import Mock

import can
import pytest
from robstride_dynamics import Motor, RobstrideBus

from motor_control_hybrid.can_transport import NonblockingRobstrideBus, LatestCommandMailbox


@pytest.fixture
def bus():
    transport = NonblockingRobstrideBus('mock', {'one': Motor(1, 'rs-04'), 'two': Motor(2, 'rs-02')}, {})
    transport.channel_handler = Mock()
    transport._motor_by_id = {1: 'one', 2: 'two'}
    yield transport
    transport.channel_handler = None


def status(device=2, host=255, kind=2, extra=0):
    return can.Message(arbitration_id=(kind << 24) | (((extra << 8) | device) << 8) | host,
                       is_extended_id=True, data=struct.pack('>HHHH', 32767, 32767, 32767, 250))


def test_enable_disable_and_mit_send_never_receive_or_wait(bus):
    bus.receive_status_frame = Mock(side_effect=AssertionError('TX must never receive'))
    bus.enable('one')
    bus.disable('one')
    bus.write_operation_frame('one', 0.2, 100, 1, 0, 0)
    assert bus.channel_handler.send.call_count == 3
    assert all(call.kwargs == {'timeout': 0.0} for call in bus.channel_handler.send.call_args_list)
    bus.channel_handler.recv.assert_not_called()


def test_mit_encoding_matches_installed_sdk(bus):
    original = RobstrideBus('mock', bus.motors, {})
    original.transmit = Mock()
    original.write_operation_frame('one', 0.2, 100, 1, 0.3, 0.4)
    bus.write_operation_frame('one', 0.2, 100, 1, 0.3, 0.4)
    kind, torque, device, data = original.transmit.call_args.args
    frame = bus.channel_handler.send.call_args.args[0]
    assert frame.arbitration_id == (kind << 24) | (torque << 8) | device
    assert frame.data == data


def test_rx_uses_actual_motor_id_and_not_send_order(bus):
    result = bus.decode_feedback(status(device=2))
    assert result.motor_name == 'two'
    assert (result.position, result.velocity, result.torque, result.temperature) == (0, 0, 0, 25)
    assert bus.decode_feedback(status(device=1)).motor_name == 'one'


@pytest.mark.parametrize('frame', [status(device=99), status(host=254), status(kind=17),
                                  can.Message(arbitration_id=1, data=bytes(8))])
def test_rx_ignores_unrelated_or_malformed_frames(bus, frame):
    assert bus.decode_feedback(frame) is None


def test_fault_and_error_frames_are_not_normal_joint_samples(bus):
    with pytest.raises(RuntimeError, match='fault report'):
        bus.decode_feedback(status(kind=21))
    with pytest.raises(RuntimeError, match='error frame'):
        bus.decode_feedback(can.Message(is_error_frame=True))


def test_mailbox_coalesces_thousands_of_targets_without_losing_enable():
    mailbox = LatestCommandMailbox()
    mailbox.put_nowait(dict(joint='a', mode=3))
    for target in range(10000):
        mailbox.put_nowait(dict(joint='a', mode=2, position=target))
    assert mailbox.qsize() == 2
    assert mailbox.get_nowait()['mode'] == 3
    assert mailbox.get_nowait()['position'] == 9999
    with pytest.raises(queue.Empty):
        mailbox.get_nowait()


def test_disable_discards_motion_and_cannot_be_overwritten_by_motion():
    mailbox = LatestCommandMailbox()
    for mode in (3, 2, 4, 2):
        mailbox.put_nowait(dict(joint='a', mode=mode))
    assert mailbox.qsize() == 1
    assert mailbox.get_nowait()['mode'] == 4


def test_socket_send_backpressure_raises_without_retry_loop(bus):
    bus.channel_handler.send.side_effect = can.CanOperationError('Transmit buffer full')
    with pytest.raises(can.CanOperationError, match='Transmit buffer full'):
        bus.write_operation_frame('one', 0, 100, 1)
    bus.channel_handler.send.assert_called_once()


def test_command_batch_is_coalesced_per_joint():
    mailbox = LatestCommandMailbox()
    for position in range(1000):
        mailbox.put_nowait([dict(joint=f'joint_{i}', mode=2, position=position) for i in range(10)])
    assert mailbox.qsize() == 10
    commands = [mailbox.get_nowait() for _ in range(10)]
    assert {c['position'] for c in commands} == {999}
    assert len({c['joint'] for c in commands}) == 10


def test_connect_sets_nonblocking_socket_and_filters_without_hardware(bus, monkeypatch):
    handler = Mock()
    factory = Mock(return_value=handler)
    bus.channel_handler = None
    monkeypatch.setattr('motor_control_hybrid.can_transport.can.ThreadSafeBus', factory)
    bus.connect()
    handler.socket.setblocking.assert_called_once_with(False)
    assert len(factory.call_args.kwargs['can_filters']) == 2
