"""Admission bridge tests with mocked ROS service; no CAN or motors."""
from types import SimpleNamespace
from concurrent.futures import Future
from unittest.mock import Mock
import pytest
from rclpy.clock import Clock
from humanoid_safety.motor_web_node import WebActionBridge
from motor_control_interfaces.msg import AbstractCommand, AbstractCommandStatus


def setup_bridge():
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = Future()
    bridge = WebActionBridge(client, Clock())
    bridge.tick()
    status = dict(estop=False, motors=[dict(joint_name='knee', online=True)])
    return bridge, client, status


@pytest.mark.parametrize('action,operation', [('home', AbstractCommand.HOME), ('zero', AbstractCommand.SET_MECHANICAL_ZERO)])
def test_typed_request_and_admission_is_not_completion(action, operation):
    bridge, client, status = setup_bridge()
    result = bridge.submit(dict(action=action, joint_name='knee'), status)
    bridge.tick()
    request = client.call_async.call_args[0][0]
    assert request.command.operation == operation
    assert request.command.joint_name == 'knee'
    assert request.command.request_id == result['request_id']
    assert request.command.header.stamp.sec > 0
    client.call_async.return_value.set_result(SimpleNamespace(accepted=True, message='admitted', status=SimpleNamespace(stage=1)))
    assert bridge.snapshot()['busy']
    assert bridge.snapshot()['result']['state'] == 'accepted'
    bridge.on_status(AbstractCommandStatus(request_id=result['request_id'], stage=AbstractCommandStatus.COMPLETED, message='verified'))
    assert not bridge.snapshot()['busy']
    assert bridge.snapshot()['result']['state'] == 'completed'


@pytest.mark.parametrize('estop,online', [(True, True), (None, True), (False, False)])
def test_no_service_call_without_live_feedback_and_released_estop(estop, online):
    bridge, client, status = setup_bridge()
    status['estop'] = estop
    status['motors'][0]['online'] = online
    with pytest.raises(ValueError):
        bridge.submit(dict(action='home', joint_name='knee'), status)
    client.call_async.assert_not_called()


def test_pending_duplicate_blocked_and_rejection_exposed():
    bridge, client, status = setup_bridge()
    bridge.submit(dict(action='zero', joint_name='knee'), status)
    with pytest.raises(ValueError):
        bridge.submit(dict(action='zero', joint_name='knee'), status)
    bridge.tick()
    client.call_async.return_value.set_result(SimpleNamespace(accepted=False, message='not commissioned', status=SimpleNamespace(stage=9)))
    assert bridge.snapshot()['result']['state'] == 'rejected'
    assert bridge.snapshot()['result']['message'] == 'not commissioned'
    assert not bridge.snapshot()['busy']


def test_response_timeout_does_not_permit_retry():
    bridge, client, status = setup_bridge()
    bridge.submit(dict(action='home', joint_name='knee'), status)
    bridge.tick()
    bridge.deadline = 0
    bridge.tick()
    assert bridge.snapshot()['result']['state'] == 'unknown'
    with pytest.raises(ValueError):
        bridge.submit(dict(action='home', joint_name='knee'), status)


def test_terminal_status_before_service_response_is_preserved():
    bridge, client, status = setup_bridge()
    result = bridge.submit(dict(action='home', joint_name='knee'), status)
    bridge.tick()
    bridge.on_status(AbstractCommandStatus(request_id=result['request_id'], stage=AbstractCommandStatus.COMPLETED, message='verified'))
    client.call_async.return_value.set_result(SimpleNamespace(accepted=True, message='admitted', status=SimpleNamespace(stage=1)))
    assert bridge.snapshot()['result']['state'] == 'completed'


def test_all_zero_advances_only_after_completion_and_stops_on_rejection():
    bridge, client, status = setup_bridge()
    status['motors'].append(dict(joint_name='ankle', online=True))
    futures = [Future(), Future()]
    client.call_async.side_effect = futures
    bridge.submit(dict(action='zero', joint_name='*'), status)
    bridge.tick()
    first = client.call_async.call_args[0][0].command
    futures[0].set_result(SimpleNamespace(accepted=True, message='admitted', status=SimpleNamespace(stage=1)))
    bridge.tick()
    assert client.call_async.call_count == 1
    bridge.on_status(AbstractCommandStatus(request_id=first.request_id, stage=6, message='verified'))
    bridge.tick()
    second = client.call_async.call_args[0][0].command
    assert second.joint_name == 'ankle' and second.request_id != first.request_id
    futures[1].set_result(SimpleNamespace(accepted=False, message='blocked', status=SimpleNamespace(stage=9)))
    result = bridge.snapshot()['result']
    assert result['completed'] == 1 and result['total'] == 2
    assert result['state'] == 'rejected' and len(result['results']) == 2
    assert not bridge.snapshot()['busy']


def test_all_home_is_one_core_owned_operation_for_all_joints():
    bridge, client, status = setup_bridge()
    status['motors'].append(dict(joint_name='ankle', online=True))
    result = bridge.submit(dict(action='home', joint_name='*'), status)
    bridge.tick()
    assert client.call_async.call_args[0][0].command.joint_name == '*'
    bridge.on_status(AbstractCommandStatus(request_id=result['request_id'], stage=6, message='all verified'))
    result = bridge.snapshot()['result']
    assert result['completed'] == result['total'] == 2
    assert [r['joint_name'] for r in result['results']] == ['knee', 'ankle']
    assert not bridge.snapshot()['busy']


def test_all_rejects_one_offline_motor_before_any_service_call():
    bridge, client, status = setup_bridge()
    status['motors'].append(dict(joint_name='ankle', online=False))
    with pytest.raises(ValueError):
        bridge.submit(dict(action='zero', joint_name='*'), status)
    client.call_async.assert_not_called()


@pytest.mark.parametrize('action,operation', [('enable', AbstractCommand.ENABLE), ('disable', AbstractCommand.DISABLE)])
def test_lifecycle_all_uses_typed_core_operation(action, operation):
    bridge, client, status = setup_bridge()
    status['motors'].append(dict(joint_name='ankle', online=True))
    result = bridge.submit(dict(action=action, joint_name='*'), status)
    bridge.tick()
    request = client.call_async.call_args[0][0].command
    assert request.operation == operation and request.joint_name == '*'
    bridge.on_status(AbstractCommandStatus(request_id=result['request_id'], stage=6, message='verified'))
    assert bridge.snapshot()['result']['completed'] == 2


def test_disable_can_preempt_pending_action_with_offline_feedback_and_estop():
    bridge, client, status = setup_bridge()
    first = Future(); second = Future(); client.call_async.side_effect = [first, second]
    old = bridge.submit(dict(action='home', joint_name='knee'), status)
    bridge.tick()
    status['estop'] = True; status['motors'][0]['online'] = False
    new = bridge.submit(dict(action='disable', joint_name='knee'), status)
    bridge.tick()
    first.set_result(SimpleNamespace(accepted=True, message='old', status=SimpleNamespace(stage=5)))
    assert bridge.snapshot()['result']['request_id'] == new['request_id'] != old['request_id']
    assert client.call_async.call_args[0][0].command.operation == AbstractCommand.DISABLE
