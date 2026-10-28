"""Exercise production inference/feedback synchronization with a fake policy.

No DDS participant, ONNX runtime, serial port or motor connection is needed.
Clocks and the policy completion time are controlled rather than timing sleeps.
"""
from collections import deque
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from motor_control_hybrid.tracking_policy_node import TrackingPolicyNode


@pytest.fixture
def node(monkeypatch):
    n = TrackingPolicyNode.__new__(TrackingPolicyNode)
    n.state_lock = threading.RLock()
    n.stopping = threading.Event()
    n.clock_values = [10.0, 100.0]
    monkeypatch.setattr('motor_control_hybrid.tracking_policy_node.time.monotonic',
                        lambda: n.clock_values[0])
    n.ros_seconds = lambda: n.clock_values[1]
    n.get_logger = lambda: Mock()
    n.rate, n.timeout, n.max_input_skew, n.max_inference = 50, .1, .02, .02
    n.policy = SimpleNamespace(
        names=[f'joint_{i}' for i in range(10)], offset=np.zeros(10), frames=299,
        kp=np.full(10, 20.), kd=np.ones(10), observation=Mock(return_value=np.zeros((1, 56))),
        run=Mock(return_value=(np.ones(10), np.full(10, .2))))
    n.previous = np.zeros(10)
    n.imu = (10.0, 99.99, np.zeros(3), np.array([0., 0., -1.]))
    n.joints = (10.0, 99.985, np.zeros(10), np.zeros(10))
    n.feedback_mode = 'joint_states'
    n.hold_first_frame = False
    n.estop, n.fault, n.started = False, None, None
    n.state = 'waiting_for_imu'
    n.frame = n.count = n.overruns = 0
    n.last_tick = None
    n.intervals, n.inference_ms, n.work_ms = (deque() for _ in range(3))
    n.angles_pub, n.actions_pub, n.command_pub = Mock(), Mock(), Mock()
    return n


def test_preserves_snapshot_stamp_and_absolute_expiry(node):
    node.tick()
    message = node.command_pub.publish.call_args.args[0]
    assert TrackingPolicyNode.stamp_seconds(message) == pytest.approx(99.985)
    expiry = message.valid_until.sec + message.valid_until.nanosec * 1e-9
    assert expiry == pytest.approx(100.085)
    np.testing.assert_array_equal(node.previous, np.ones(10))
    assert node.count == 1


def test_slow_inference_is_discarded_and_does_not_advance_history(node):
    def slow(*args):
        node.clock_values[:] = [10.03, 100.03]
        return np.ones(10), np.ones(10)
    node.policy.run = slow
    node.tick()
    assert node.fault == 'Inference result expired'
    assert node.expiry_diagnostic['causes'] == ['control_budget']
    assert node.expiry_diagnostic['inference_ms'] == pytest.approx(30)
    assert node.expiry_diagnostic['result_lock_wait_ms'] == pytest.approx(0)
    node.command_pub.publish.assert_not_called()
    np.testing.assert_array_equal(node.previous, np.zeros(10))


def test_result_lock_wait_is_reported_separately_from_inference(node):
    class DelayedResultLock:
        def __init__(self):
            self.lock = threading.RLock()
            self.entries = 0

        def __enter__(self):
            self.entries += 1
            if self.entries == 2:
                node.clock_values[:] = [10.025, 100.025]
            self.lock.acquire()
            return self

        def __exit__(self, *args):
            self.lock.release()

    node.state_lock = DelayedResultLock()
    node.tick()
    assert node.fault == 'Inference result expired'
    assert node.expiry_diagnostic['causes'] == ['control_budget']
    assert node.expiry_diagnostic['inference_ms'] == pytest.approx(0)
    assert node.expiry_diagnostic['result_lock_wait_ms'] == pytest.approx(25)
    node.command_pub.publish.assert_not_called()


def test_reference_end_does_not_expire_a_fresh_result(node):
    node.started = 3.0  # Seven seconds elapsed; the 299-frame clip has ended.
    node.tick()
    assert node.frame == 298
    assert node.fault is None
    node.command_pub.publish.assert_called_once()


def test_new_feedback_cannot_make_old_snapshot_fresh(node):
    node.imu = (9.91, 99.91, *node.imu[2:])
    node.joints = (9.91, 99.91, *node.joints[2:])
    def run(*args):
        node.clock_values[:] = [10.015, 100.015]
        node.imu = (10.015, 100.015, *node.imu[2:])
        node.joints = (10.015, 100.015, *node.joints[2:])
        return np.ones(10), np.ones(10)
    node.policy.run = run
    node.tick()
    assert node.fault == 'Inference result expired'
    assert node.expiry_diagnostic['causes'] == ['imu_stale', 'joints_stale']
    assert node.expiry_diagnostic['imu_source_age_ms'] == pytest.approx(105)
    assert node.expiry_diagnostic['joint_source_age_ms'] == pytest.approx(105)
    node.command_pub.publish.assert_not_called()


def test_skewed_inputs_wait_without_running_policy(node):
    node.joints = (10., 99.95, *node.joints[2:])
    node.tick()
    assert node.state == 'waiting_for_aligned_inputs'
    node.policy.run.assert_not_called()
    node.command_pub.publish.assert_not_called()


def test_callbacks_and_estop_continue_while_inference_is_blocked(node):
    entered, release = threading.Event(), threading.Event()
    def blocked(*args):
        entered.set()
        assert release.wait(2)
        return np.ones(10), np.ones(10)
    node.policy.run = blocked
    worker = threading.Thread(target=node.tick)
    worker.start()
    try:
        assert entered.wait(2)
        msg = JointState(name=node.policy.names, position=[.3]*10, velocity=[0.]*10)
        msg.header.stamp.sec = 100
        # A separate callback must complete before inference is released.
        callback = threading.Thread(target=lambda: (node.on_joints(msg), node.on_estop(Bool(data=True))))
        callback.start()
        callback.join(timeout=1)
        assert not callback.is_alive()
        assert node.joints[2][0] == pytest.approx(.3)
        assert node.fault == 'estop'
    finally:
        release.set()
        worker.join(timeout=2)
    node.command_pub.publish.assert_not_called()


def test_shutdown_during_inference_suppresses_publish(node):
    def run(*args):
        node.stopping.set()
        return np.ones(10), np.ones(10)
    node.policy.run = run
    node.tick()
    node.command_pub.publish.assert_not_called()


def test_nonfinite_output_latches_fault(node):
    node.policy.run.return_value = np.full(10, np.nan), np.ones(10)
    node.tick()
    assert 'Non-finite policy output' in node.fault
    node.command_pub.publish.assert_not_called()


def test_imu_filter_uses_capture_time_and_rejects_reordered_samples(node):
    from sensor_msgs.msg import Imu
    node.rotation, node.bias = np.eye(3), np.zeros(3)
    node.calibrated, node.imu_frame = True, 'imu_link'
    node.orientation_source = 'complementary'
    node.rejected_imu_timestamps = 0
    node.filter = Mock()
    node.filter.update.return_value = np.array([0., 0., -1.])
    msg = Imu()
    msg.header.stamp.sec = 100
    msg.header.frame_id = 'imu_link'
    msg.linear_acceleration.z = 9.80665
    node.on_imu(msg)
    assert node.filter.update.call_args.args[2] == 100.0
    accepted = node.imu
    node.on_imu(msg)  # Duplicate source stamp must not re-integrate the gyro.
    assert node.imu is accepted
    assert node.filter.update.call_count == 1
    assert node.rejected_imu_timestamps == 1
