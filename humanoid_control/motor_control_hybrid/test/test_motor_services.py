"""Typed management services with cached CAN fixtures; no ROS/CAN hardware."""
import math
import threading
from unittest.mock import Mock, patch

import pytest
from motor_control_interfaces.srv import ExecuteMotorOperation, GetMotorFeedback
from motor_control_hybrid.can_transport import MotorFeedback
from motor_control_hybrid.python_can_node import PythonCanNode


@pytest.fixture
def node():
    node = PythonCanNode.__new__(PythonCanNode)
    node.motor_identity = {'knee': dict(joint_name='knee', can_interface='can1',
                                      motor_id=4, model='rs-04')}
    node.state_lock = threading.Lock()
    node.raw_feedback = {}
    node.feedback_timeout_s = .1
    node.estop_active = threading.Event()
    node.transport_fault = threading.Event()
    node.bus_by_joint = {'knee': Mock()}
    return node


def read(node, name='knee'):
    return node._get_feedback_callback(GetMotorFeedback.Request(joint_name=name),
                                       GetMotorFeedback.Response())


def cached(node, mode=2, faults=0):
    node.raw_feedback['knee'] = (MotorFeedback('motor_4', .2, .01, .3, 25., mode, faults, 100.),
                                 10., 100.)


def test_missing_and_unknown_feedback_is_not_zero(node):
    result = read(node)
    assert not result.success and not result.feedback.available
    assert math.isnan(result.feedback.position_rad)
    assert result.feedback.motor_id == 4
    assert not read(node, 'unknown').success
    assert not read(node, 'unknown').feedback.available


def test_raw_feedback_preserves_coordinates_and_status_mode(node):
    cached(node)
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.02):
        result = read(node)
    assert result.success and result.feedback.available and result.feedback.fresh
    assert result.feedback.position_rad == .2
    assert result.feedback.status_mode == 2  # Status Motor/run, not velocity run_mode.
    assert result.feedback.stamp.sec == 100
    assert result.feedback.age_s == pytest.approx(.02)
    node.bus_by_joint['knee'].assert_not_called()
    assert not node.bus_by_joint['knee'].mock_calls


def test_stale_sample_retains_values_but_is_not_success(node):
    cached(node)
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.2):
        result = read(node)
    assert not result.success and not result.feedback.fresh
    assert result.feedback.available and result.feedback.position_rad == .2


@pytest.mark.parametrize('faults,transport_fault', [(1, False), (0, True)])
def test_fresh_feedback_does_not_hide_latched_or_reported_fault(node, faults, transport_fault):
    cached(node, faults=faults)
    if transport_fault:
        node.transport_fault.set()
    with patch('motor_control_hybrid.python_can_node.time.monotonic', return_value=10.02):
        result = read(node)
    assert result.feedback.fresh and not result.success


@pytest.mark.parametrize('request_id,joint,operation', [
    ('zero-1', 'knee', ExecuteMotorOperation.Request.SET_MECHANICAL_ZERO),
    ('zero-1', 'knee', 6),  # Raw CAN frame numbers are not ROS operations.
    ('zero-1', 'knee', 2), ('zero-1', 'missing', 1), ('', 'knee', 1),
    ('x' * 129, 'knee', 1)])
def test_zero_endpoint_never_transmits_without_transaction_support(node, request_id, joint, operation):
    response = node._operation_callback(ExecuteMotorOperation.Request(
        request_id=request_id, joint_name=joint, operation=operation), ExecuteMotorOperation.Response())
    assert not response.accepted and response.state == response.REJECTED
    assert response.request_id == request_id and response.message
    assert not node.bus_by_joint['knee'].mock_calls


def test_zero_endpoint_explicitly_reports_missing_workflow(node):
    response = node._operation_callback(ExecuteMotorOperation.Request(
        request_id='zero-1', joint_name='knee', operation=1), ExecuteMotorOperation.Response())
    assert 'pause/exclusive lease' in response.message
    assert 'no CAN command sent' in response.message
