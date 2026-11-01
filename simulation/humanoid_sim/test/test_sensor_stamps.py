"""Sensor publication preserves acquisition time through delayed ROS callbacks."""
from types import SimpleNamespace
from unittest.mock import Mock
import threading
from rclpy._rclpy_pybind11 import RCLError

import numpy as np
from humanoid_sim.imu_nodes import SimBodyImuNode
from humanoid_sim.motor_backend_node import SimMotorBackendNode
from humanoid_sim.main import SimControlNode


def unexpected_clock_read():
    raise AssertionError('callback time must not replace acquisition time')


def test_imu_uses_physics_source_stamp():
    sample = SimpleNamespace(stamp_ns=123_000_000_456, quat_wxyz=np.array([1., 0, 0, 0]))
    node = SimpleNamespace(
        world=SimpleNamespace(control_fault=None, body_imu=lambda: sample),
        _noise=SimpleNamespace(apply=lambda _: (np.zeros(3), np.zeros(3))),
        _max_sample_wait_s=0., _last_publish_mono=None, _max_publish_gap_s=0.,
        _last_callback_mono=None, _max_callback_gap_s=0., _max_publish_call_s=0.,
        _frame_id='imu_link', imu_pub=Mock(), _published=0,
        first_sample=threading.Event(), get_clock=unexpected_clock_read)
    SimBodyImuNode._publish(node)
    stamp = node.imu_pub.publish.call_args.args[0].header.stamp
    assert (stamp.sec, stamp.nanosec) == (123, 456)
    assert node.first_sample.is_set()


def test_imu_publish_failure_drops_frame_then_samples_again():
    sample = SimpleNamespace(stamp_ns=123_000_000_456, quat_wxyz=np.array([1., 0, 0, 0]))
    node = SimpleNamespace(
        world=SimpleNamespace(control_fault=None, body_imu=Mock(return_value=sample)),
        _noise=SimpleNamespace(apply=lambda _: (np.zeros(3), np.zeros(3))),
        _max_sample_wait_s=0., _last_publish_mono=None, _max_publish_gap_s=0.,
        _last_callback_mono=None, _max_callback_gap_s=0., _max_publish_call_s=0.,
        _frame_id='imu_link', imu_pub=Mock(), _published=0,
        first_sample=threading.Event(), get_logger=lambda: Mock())
    node.imu_pub.publish.side_effect = [RCLError('injected DDS failure'), None]
    SimBodyImuNode._publish(node)
    assert not node.first_sample.is_set()
    assert node._published == 0
    sample.stamp_ns += 10_000_000
    SimBodyImuNode._publish(node)
    assert node.world.body_imu.call_count == 2
    assert node._published == 1
    assert node.first_sample.is_set()
    assert node._publish_failures['/imu/data_raw']['count'] == 1
    stamp = node.imu_pub.publish.call_args.args[0].header.stamp
    assert stamp.sec*10**9+stamp.nanosec == sample.stamp_ns


def test_joint_feedback_preserves_source_and_monotonic_acquisition():
    sample = SimpleNamespace(stamp_ns=123_000_000_456, sampled_mono=12.,
                             names=['joint'], position=[.2], velocity=[0.], effort=[1.])
    node = SimpleNamespace(
        world=SimpleNamespace(control_fault=None, joint_snapshot=lambda: sample,
                              lock=threading.RLock(), motors=SimpleNamespace(pop_events=lambda: [])),
        joint_state_pub=Mock(), first_feedback=threading.Event(),
        _timing={'rx_frames': 0}, _feedback_msg=None,
        get_clock=unexpected_clock_read)
    SimMotorBackendNode._publish_states(node)
    msg = node.joint_state_pub.publish.call_args.args[0]
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (123, 456)
    assert node._last_sample_mono == 12.
    assert node._last_feedback_msg is msg


def test_joint_write_failure_does_not_mark_feedback_ready_or_cache_failed_frame():
    sample = SimpleNamespace(stamp_ns=123_000_000_456, sampled_mono=12.,
                             names=['joint'], position=[.2], velocity=[0.], effort=[1.])
    node = SimpleNamespace(
        world=SimpleNamespace(control_fault=None, joint_snapshot=lambda: sample,
                              lock=threading.RLock(), motors=SimpleNamespace(pop_events=lambda: [])),
        joint_state_pub=Mock(), first_feedback=threading.Event(),
        _timing={'rx_frames': 0}, _feedback_msg=None, get_logger=lambda: Mock())
    node.joint_state_pub.publish.side_effect = [RCLError('injected'), None]
    SimMotorBackendNode._publish_states(node)
    assert not node.first_feedback.is_set()
    assert node._timing['rx_frames'] == 0
    sample.stamp_ns += 20_000_000
    SimMotorBackendNode._publish_states(node)
    assert node.first_feedback.is_set()
    assert node._timing['rx_frames'] == 1
    assert node._publish_failures['/joint_states']['count'] == 1


def test_pose_and_contacts_share_acquisition_stamp():
    world = SimpleNamespace(lock=threading.RLock(), sample_stamp_ns=123_000_000_456,
        base_state=lambda: (np.zeros(3), np.array([1., 0, 0, 0]), np.zeros(3), np.zeros(3)),
        foot_contacts=lambda: (np.zeros(2), np.zeros(2)))
    node = SimpleNamespace(world=world, odom_pub=Mock(), feet_pub=Mock(),
                           _odom_msg=None, _feet_msg=None,
                           get_clock=unexpected_clock_read)
    SimControlNode._publish_odom(node)
    odom = node.odom_pub.publish.call_args.args[0]
    feet = node.feet_pub.publish.call_args.args[0]
    assert odom.header.stamp == feet.header.stamp
    assert (odom.header.stamp.sec, odom.header.stamp.nanosec) == (123, 456)
