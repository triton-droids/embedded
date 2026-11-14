"""Head (camera-rig) IMU: every physics sample once, acquisition stamps, split publisher."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from humanoid_sim.imu_nodes import _RateGate
from humanoid_sim.imu_split_node import SplitHeadImuNode

STEP_NS = 5_000_000  # 5 ms physics step


def test_gate_passes_every_physics_sample_once_at_the_physics_rate():
    gate = _RateGate(200.0)
    polled = [0, 0, STEP_NS, STEP_NS, 2*STEP_NS + 300_000, 3*STEP_NS, 3*STEP_NS]
    assert [gate.accept(s) for s in polled] == [True, False, True, False, True, True, False]


def test_gate_decimates_to_a_lower_rate():
    gate = _RateGate(100.0)
    assert [gate.accept(i*STEP_NS) for i in range(5)] == [True, False, True, False, True]


def _node(record):
    return SimpleNamespace(
        reader=SimpleNamespace(read=lambda: record), _gate=_RateGate(200.0),
        _noise=SimpleNamespace(apply=lambda s: (s.gyro, s.accel)),
        _frame_id='head_imu_link', imu_pub=Mock(), published=0)


def _record(**changes):
    record = dict(stamp_ns=123_000_000_456, valid=True, head_valid=True, control_fault=False,
                  head_gyro=(0.1, 0.2, 0.3), head_accel=(0.0, -4.9, 8.5))
    record.update(changes)
    return record


def test_split_head_imu_publishes_the_sample_with_its_physics_stamp():
    node = _node(_record())
    SplitHeadImuNode._publish(node)
    SplitHeadImuNode._publish(node)  # Same record again: not republished.
    assert node.imu_pub.publish.call_count == 1
    msg = node.imu_pub.publish.call_args.args[0]
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (123, 456)
    assert msg.header.frame_id == 'head_imu_link'
    assert msg.orientation_covariance[0] == -1.0
    assert np.allclose([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                       [0.1, 0.2, 0.3])
    assert msg.linear_acceleration.y == -4.9


def test_split_head_imu_is_silent_without_head_data_or_under_fault():
    for record in (_record(head_valid=False), _record(control_fault=True), None):
        node = _node(record)
        SplitHeadImuNode._publish(node)
        node.imu_pub.publish.assert_not_called()
