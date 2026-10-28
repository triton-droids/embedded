"""Exercise delayed discovery and release failures without publishing to ROS."""
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('motor_control_interfaces.msg')

from builtin_interfaces.msg import Time  # noqa: E402
from motor_control_interfaces.msg import MotorCommand  # noqa: E402
from humanoid_sim.autostart_node import SimAutostartNode  # noqa: E402


@pytest.fixture
def node():
    clock = Mock()
    clock.now.return_value.nanoseconds = 10_000_000_000
    clock.now.return_value.to_msg.return_value = Time(sec=10)
    n = SimpleNamespace(
        _done=False, _release_timer=None, _first_policy_at=None, _request_at=None,
        _enabled_joints=[], _waiting_logged=False, _startup_timeout=2.0,
        _release_delay=0.1, _hold_client=Mock(), _command_pub=Mock(),
        _policy_sub=Mock(), get_clock=Mock(return_value=clock),
        get_logger=Mock(return_value=Mock()), create_timer=Mock(),
        destroy_subscription=Mock())
    n._hold_client.service_is_ready.return_value = False
    n._command_pub.get_subscription_count.return_value = 1
    for name in ('_on_policy', '_release', '_released', '_fail', '_check_timeout'):
        setattr(n, name, MethodType(getattr(SimAutostartNode, name), n))
    return n


def policy_command():
    msg = MotorCommand()
    msg.header.stamp = Time(sec=10)
    msg.valid_until = Time(sec=10, nanosec=100_000_000)
    msg.joint_name = ['left_hip1_joint']
    msg.mode = [MotorCommand.MODE_MOTION]
    return msg


def start(node):
    node._hold_client.service_is_ready.return_value = True
    node._on_policy(policy_command())
    assert list(node._command_pub.publish.call_args.args[0].mode) == [MotorCommand.MODE_ENABLE]


def test_late_discovery_waits_for_new_fresh_command(node):
    node._on_policy(policy_command())
    node._command_pub.publish.assert_not_called()
    node.destroy_subscription.assert_not_called()
    node._hold_client.service_is_ready.return_value = True
    expired = policy_command()
    expired.valid_until = Time(sec=9)
    node._on_policy(expired)
    node._command_pub.publish.assert_not_called()
    node._on_policy(policy_command())
    assert node._command_pub.publish.call_count == 1
    node._on_policy(policy_command())
    assert node._command_pub.publish.call_count == 1


def test_wait_for_motor_subscription(node):
    node._hold_client.service_is_ready.return_value = True
    node._command_pub.get_subscription_count.return_value = 0
    node._on_policy(policy_command())
    node._command_pub.publish.assert_not_called()


def test_discovery_timeout_never_enables(node):
    node._on_policy(policy_command())
    node._first_policy_at -= 3.0
    node._check_timeout()
    node._hold_client.service_is_ready.return_value = True
    node._on_policy(policy_command())
    assert node._done
    node._command_pub.publish.assert_not_called()


def test_service_disappears_disables_enabled_motors(node):
    start(node)
    node._hold_client.service_is_ready.return_value = False
    node._release()
    assert node._done
    assert list(node._command_pub.publish.call_args.args[0].mode) == [MotorCommand.MODE_DISABLE]
    node._hold_client.call_async.assert_not_called()


@pytest.mark.parametrize('result', ['rejected', 'exception', 'timeout'])
def test_release_failure_disables_and_does_not_rearm(node, result):
    start(node)
    node._release()
    future = Mock()
    if result == 'rejected':
        future.result.return_value = SimpleNamespace(success=False)
        node._released(future)
    elif result == 'exception':
        future.result.side_effect = RuntimeError('lost service')
        node._released(future)
    else:
        node._request_at -= 3.0
        node._check_timeout()
        future.result.return_value = SimpleNamespace(success=True)
        node._released(future)
    assert node._done
    assert list(node._command_pub.publish.call_args.args[0].mode) == [MotorCommand.MODE_DISABLE]
    count = node._command_pub.publish.call_count
    node._on_policy(policy_command())
    assert node._command_pub.publish.call_count == count


def test_release_success(node):
    start(node)
    node._release()
    assert node._hold_client.call_async.call_args.args[0].data is False
    future = Mock()
    future.result.return_value = SimpleNamespace(success=True)
    node._released(future)
    assert node._done
    assert node._command_pub.publish.call_count == 1


def test_release_waits_for_continuing_fresh_commands(node):
    start(node)
    node.get_clock().now.return_value.nanoseconds = 10_080_000_000
    node._release()
    node._hold_client.call_async.assert_not_called()
    node._release_timer.cancel.assert_not_called()
    fresh = policy_command()
    fresh.header.stamp = Time(sec=10, nanosec=80_000_000)
    fresh.valid_until = Time(sec=10, nanosec=180_000_000)
    node._on_policy(fresh)
    node._release()
    node._hold_client.call_async.assert_called_once()
    assert node._command_pub.publish.call_count == 1  # Never repeat ENABLE.


def test_stopped_policy_aborts_with_gantry_held(node):
    start(node)
    node.get_clock().now.return_value.nanoseconds = 11_000_000_000
    node._release()
    node._hold_client.call_async.assert_not_called()
    node._first_policy_at -= 3
    node._check_timeout()
    assert node._done
    assert list(node._command_pub.publish.call_args.args[0].mode) == [MotorCommand.MODE_DISABLE]


def test_backend_not_ready_retries_without_enabling_again(node):
    start(node)
    node._release()
    future = Mock()
    future.result.return_value = SimpleNamespace(
        success=False, message='Release denied: all motors need fresh motion commands')
    node._released(future)
    assert not node._done
    assert node._request_at is None
    node._release_timer.reset.assert_called_once()
    assert node._command_pub.publish.call_count == 1
