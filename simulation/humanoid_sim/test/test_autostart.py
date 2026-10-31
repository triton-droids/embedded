"""Exercise delayed discovery and release failures without publishing to ROS."""
from types import MethodType, SimpleNamespace
from collections import deque
from unittest.mock import Mock
import time

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
        destroy_subscription=Mock(), _use_control_core=False)
    n._hold_client.service_is_ready.return_value = False
    n._command_pub.get_subscription_count.return_value = 1
    for name in ('_on_policy', '_release', '_released', '_fail', '_check_timeout'):
        setattr(n, name, MethodType(getattr(SimAutostartNode, name), n))
    return n


@pytest.fixture
def verified_node(node):
    from rclpy.time import Time as RosTime
    node._use_control_core = True
    node._enable_id = 'test-verified-enable'
    node._enable_pending = False
    node._enable_accepted = False
    node._enable_request_at = None
    node._policy_sources = deque(maxlen=128)
    node._forwarded_deadline_ns = 10_100_000_000
    node._forwarded_ready_since = time.monotonic() - 1.
    node._last_forwarded_at = time.monotonic()
    node._enable_client = Mock()
    node._enable_client.service_is_ready.return_value = True
    node.get_clock().now.return_value = RosTime(nanoseconds=10_000_000_000)
    for name in ('_start_verified', '_enable_reply', '_on_enable_status', '_on_forwarded'):
        setattr(node, name, MethodType(getattr(SimAutostartNode, name), node))
    return node


def test_verified_enable_does_not_publish_unacknowledged_enable_or_release(verified_node):
    from motor_control_interfaces.msg import AbstractCommand, AbstractCommandStatus
    node = verified_node
    node._hold_client.service_is_ready.return_value = True
    node._on_policy(policy_command())
    request = node._enable_client.call_async.call_args.args[0]
    assert request.command.operation == AbstractCommand.ENABLE
    node._command_pub.publish.assert_not_called()


    assert node._release_timer is None
    future = Mock()
    future.result.return_value = SimpleNamespace(accepted=True)
    node._enable_reply(future)
    assert node._release_timer is None
    for _ in range(5):
        node._on_policy(policy_command())
    assert node._enable_client.call_async.call_count == 1
    status = AbstractCommandStatus()
    status.request_id = 'unrelated'
    status.stage = AbstractCommandStatus.COMPLETED
    node._on_enable_status(status)
    assert node._release_timer is None
    status.request_id = node._enable_id
    node._on_enable_status(status)
    assert node._release_timer is not None
    node._command_pub.publish.assert_not_called()


def test_release_requires_cpp_forwarded_policy_source_not_idle_motion(verified_node):
    node = verified_node
    node._forwarded_deadline_ns = 0
    node._hold_client.service_is_ready.return_value = True
    node._policy_sources.append(10_000_000_000)
    node._latest_policy_deadline_ns = 10_100_000_000
    node._release_timer = Mock()
    node._enabled_joints = ['left_hip1_joint']
    node._release()
    node._hold_client.call_async.assert_not_called()
    idle = policy_command()
    idle.header.stamp = Time(sec=10, nanosec=1000_000)
    node._on_forwarded(idle)
    node._release()
    node._hold_client.call_async.assert_not_called()
    forwarded = policy_command()
    node._on_forwarded(forwarded)
    node._release()
    node._hold_client.call_async.assert_called_once()


def test_rejected_admission_can_retry_but_not_an_accepted_enable(verified_node):
    node = verified_node
    node._hold_client.service_is_ready.return_value = True
    node._on_policy(policy_command())
    future = Mock()
    future.result.return_value = SimpleNamespace(accepted=False, message='feedback not ready')
    node._enable_reply(future)
    node._on_policy(policy_command())
    assert node._enable_client.call_async.call_count == 2
    future.result.return_value = SimpleNamespace(accepted=True)
    node._enable_reply(future)
    node._on_policy(policy_command())
    assert node._enable_client.call_async.call_count == 2


def test_ambiguous_enable_timeout_disables_without_rearming(verified_node):
    node = verified_node
    node._hold_client.service_is_ready.return_value = True
    node._on_policy(policy_command())
    node._first_policy_at -= 3
    node._check_timeout()
    assert node._done
    assert list(node._command_pub.publish.call_args.args[0].mode) == [MotorCommand.MODE_DISABLE]
    node._on_policy(policy_command())
    assert node._enable_client.call_async.call_count == 1


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


def test_verified_enable_waits_for_end_to_end_forwarding_before_arming(verified_node):
    node = verified_node
    node._forwarded_deadline_ns = 0
    node._hold_client.service_is_ready.return_value = True
    sample = policy_command()
    node._on_policy(sample)
    node._enable_client.call_async.assert_not_called()
    node._on_forwarded(sample)
    node._forwarded_ready_since -= .31
    node._on_policy(sample)
    node._enable_client.call_async.assert_called_once()
    accepted = Mock()
    accepted.result.return_value = SimpleNamespace(accepted=True)
    node._enable_reply(accepted)
    assert node._forwarded_deadline_ns == 0  # Require new forwarding after enable too.


def test_preflight_requires_sustained_forwarding_not_one_good_frame(verified_node):
    node = verified_node
    node._hold_client.service_is_ready.return_value = True
    node._forwarded_deadline_ns = 0
    node._forwarded_ready_since = None
    node._last_forwarded_at = None
    sample = policy_command()
    node._on_policy(sample)
    node._on_forwarded(sample)
    node._on_policy(sample)
    node._enable_client.call_async.assert_not_called()
    node._forwarded_ready_since -= .31
    node._on_policy(sample)
    node._enable_client.call_async.assert_called_once()
