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


def _node(record, samples):
    return SimpleNamespace(
        reader=SimpleNamespace(read_head_since=lambda after: None if record is None else (
            record, [x for x in samples if x['index'] > after])),
        _gate=_RateGate(200.0), _noise=SimpleNamespace(apply=lambda s: (s.gyro, s.accel)),
        _frame_id='head_imu_link', imu_pub=Mock(), published=0, missed=0, _last_index=None)


def _record(**changes):
    record = dict(valid=True, head_valid=True, control_fault=False)
    record.update(changes)
    return record


def _samples(first, last):
    return [dict(index=i, stamp_ns=123_000_000_456 + i*STEP_NS, gyro=(0.1, 0.2, 0.3),
                 accel=(0.0, -4.9, 8.5)) for i in range(first, last + 1)]


def _stamps(node):
    return [c.args[0].header.stamp.sec*10**9 + c.args[0].header.stamp.nanosec
            for c in node.imu_pub.publish.call_args_list]


def test_split_head_imu_publishes_every_new_sample_once_with_its_physics_stamp():
    samples = _samples(1, 3)
    node = _node(_record(), samples)
    SplitHeadImuNode._publish(node)  # First wake-up: newest sample only.
    samples += _samples(4, 5)         # Two physics steps per 10 ms wake-up.
    SplitHeadImuNode._publish(node)
    SplitHeadImuNode._publish(node)  # Nothing new: nothing republished.
    assert _stamps(node) == [123_000_000_456 + i*STEP_NS for i in (3, 4, 5)]
    msg = node.imu_pub.publish.call_args.args[0]
    assert msg.header.frame_id == 'head_imu_link'
    assert msg.orientation_covariance[0] == -1.0
    assert np.allclose([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                       [0.1, 0.2, 0.3])
    assert msg.linear_acceleration.y == -4.9
    assert node.missed == 0


def test_split_head_imu_counts_samples_lost_from_the_ring():
    samples = _samples(1, 1)
    node = _node(_record(), samples)
    SplitHeadImuNode._publish(node)
    samples[:] = _samples(40, 41)
    SplitHeadImuNode._publish(node)
    assert node.missed == 38 and node.published == 3


def test_split_head_imu_is_silent_without_head_data_or_under_fault():
    for record in (_record(head_valid=False), _record(control_fault=True), None):
        node = _node(record, _samples(1, 2))
        SplitHeadImuNode._publish(node)
        node.imu_pub.publish.assert_not_called()
