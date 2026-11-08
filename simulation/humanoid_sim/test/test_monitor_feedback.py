"""Monitoring must not refresh cached feedback or continue after physics freezes."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import JointState
from humanoid_sim.motor_backend_node import SimMotorBackendNode


def test_monitor_reuses_source_stamp_and_stops_on_control_fault():
    sample = JointState()
    sample.header.stamp.sec = 12
    sample.name = ['joint']
    sample.position = [0.2]
    node = SimpleNamespace(world=SimpleNamespace(control_fault=None),
                           _last_feedback_msg=sample, monitor_state_pub=Mock())
    SimMotorBackendNode._publish_monitor_state(node)
    sent = node.monitor_state_pub.publish.call_args.args[0]
    assert sent.header.stamp.sec == 12
    assert list(sent.position) == [0.2]
    node.monitor_state_pub.reset_mock()
    node.world.control_fault = 'command expired'
    SimMotorBackendNode._publish_monitor_state(node)
    node.monitor_state_pub.publish.assert_not_called()


def test_monitor_waits_for_first_feedback():
    node = SimpleNamespace(world=SimpleNamespace(control_fault=None),
                           _last_feedback_msg=None, monitor_state_pub=Mock())
    SimMotorBackendNode._publish_monitor_state(node)
    node.monitor_state_pub.publish.assert_not_called()
