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


@pytest.mark.parametrize('position,velocity,torque,kp,kd,ident,payload', [
    (-4*3.141592653589793, -15, -120, 0, 0, 0x01000001, '0000000000000000'),
    (0, 0, 0, 0, 0, 0x017FFF01, '7fff7fff00000000'),
    (4*3.141592653589793, 15, 120, 5000, 100, 0x01FFFF01, 'ffffffffffffffff'),
    (2*3.141592653589793, 7.5, 60, 2500, 50, 0x01BFFF01, 'bfffbfff7fff7fff'),
])
def test_rs04_independent_manual_vectors(bus, position, velocity, torque, kp, kd, ident, payload):
    bus.write_operation_frame('one', position, kp, kd, velocity, torque)
    frame = bus.channel_handler.send.call_args.args[0]
    assert frame.is_extended_id and not frame.is_fd and frame.dlc == 8
    assert frame.arbitration_id == ident
    assert bytes(frame.data) == bytes.fromhex(payload)


def test_rx_uses_actual_motor_id_and_not_send_order(bus):
    result = bus.decode_feedback(status(device=2))
    assert result.motor_name == 'two'
    # Zero lies between two codes; at 0x7fff errors stay within half a quantum.
    assert abs(result.position) <= 4*3.141592653589793/65535 + 1e-12
    assert abs(result.velocity) <= 44/65535 + 1e-12
    assert abs(result.torque) <= 17/65535 + 1e-12
    assert result.temperature == 25
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


def test_type2_firmware_reply_is_never_joint_feedback(bus):
    frame = status(device=1)
    frame.data = bytearray(b'\x00\xc4\x56\x01\x02\x03\x04\x00')
    assert bus.decode_feedback(frame) is None


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


def test_lifecycle_exact_identifier_payload_and_no_fault_clear(bus):
    bus.enable('one'); bus.disable('two')
    frames = [c.args[0] for c in bus.channel_handler.send.call_args_list]
    assert [(f.arbitration_id, bytes(f.data)) for f in frames] == [(0x0300FF01, bytes(8)), (0x0400FF02, bytes(8))]


@pytest.mark.parametrize('model,vmax,tmax,kpmax,kdmax', [('rs-02',44,17,500,5), ('rs-03',20,60,5000,100), ('rs-04',15,120,5000,100)])
def test_deployed_model_ranges_and_feedback_endpoints(model,vmax,tmax,kpmax,kdmax):
    import math
    tx = NonblockingRobstrideBus('mock', {'motor': Motor(10, model)}, {})
    tx.channel_handler = Mock(); tx._motor_by_id = {10:'motor'}
    tx.write_operation_frame('motor', 4*math.pi, kpmax, kdmax, vmax, tmax)
    frame = tx.channel_handler.send.call_args.args[0]
    assert frame.arbitration_id == 0x01FFFF0A and bytes(frame.data) == bytes.fromhex('ffffffffffffffff')
    received = can.Message(arbitration_id=0x02800AFF, is_extended_id=True, data=bytes.fromhex('ffffffffffff012c'))
    sample = tx.decode_feedback(received)
    assert sample.position == 4*math.pi and sample.velocity == vmax and sample.torque == tmax
    assert sample.temperature == 30 and sample.mode == 2 and sample.fault_bits == 0
    tx.channel_handler = None


@pytest.mark.parametrize('value', [float('nan'),float('inf'),float('-inf')])
def test_nonfinite_mit_inputs_rejected_without_transmit(bus,value):
    with pytest.raises(ValueError):bus.write_operation_frame('one',value,1,1)
    bus.channel_handler.send.assert_not_called()


def test_fd_and_dlc_mismatch_are_not_classic_status(bus):
    frame=status();frame.is_fd=True
    assert bus.decode_feedback(frame) is None
    frame=status();frame.dlc=7
    assert bus.decode_feedback(frame) is None

@pytest.mark.parametrize('kind,payload', [('mode','0570000002000000'), ('current','187000000000a040'), ('acceleration','22700000cdcc4c3e'), ('velocity','0a700000000000bf')])
def test_parameter_write_little_endian_reference_vectors(bus,kind,payload):
    from motor_control_hybrid.python_can_node import PythonCanNode
    from robstride_dynamics import ParameterType
    node=PythonCanNode.__new__(PythonCanNode)
    parameters={'mode':(ParameterType.MODE,2),'current':(ParameterType.CURRENT_LIMIT,5.),
                'acceleration':(ParameterType.VEL_ACCELERATION_TARGET,.2),'velocity':(ParameterType.VELOCITY_TARGET,-.5)}
    parameter,value=parameters[kind]
    node._write_parameter(bus,'joint','one',parameter,value)
    frame=bus.channel_handler.send.call_args.args[0]
    assert frame.arbitration_id==0x1200FF01
    assert bytes(frame.data)==bytes.fromhex(payload)


def test_home_position_frame_does_not_treat_speed_limit_as_velocity_feedforward(bus):
    from motor_control_hybrid.python_can_node import PythonCanNode
    node=PythonCanNode.__new__(PythonCanNode)
    node.current_mode_by_joint={'joint':0}
    node.max_vel_rad_s=4.5;node.default_kp=2.;node.default_kd=.2
    node._send_position_command(bus,'joint','one',dict(position=-.2,velocity=.1,kp=2.,kd=.2))
    frame=bus.channel_handler.send.call_args.args[0]
    assert bytes(frame.data[2:4])==bytes.fromhex('7fff')
