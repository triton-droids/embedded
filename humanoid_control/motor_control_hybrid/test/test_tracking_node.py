"""Exercise callback faults and output gating without any serial or motor I/O."""
from pathlib import Path
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('motor_control_interfaces.msg')
pytest.importorskip('onnxruntime')
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from motor_control_hybrid.tracking_policy_node import TrackingPolicyNode  # noqa: E402


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node(monkeypatch):
    # The node loads its default (installed) model; this is the same export
    # shipped in the repository, so gate on that copy rather than a developer path.
    model = (Path(__file__).resolve().parents[1] / 'models/20260914_170827/20260914_170827.onnx')
    if not model.exists():
        pytest.skip('Deployed policy export absent')
    rclpy.init(args=['--ros-args', '-p', 'calibration_seconds:=0.0'])
    monkeypatch.setattr(TrackingPolicyNode, "run_worker", lambda self: None)
    policy_node = TrackingPolicyNode()
    policy_node.angles_pub = Capture()
    policy_node.actions_pub = Capture()
    policy_node.command_pub = Capture()
    yield policy_node
    policy_node.destroy_node()
    rclpy.shutdown()


def imu_sample(node):
    imu = Imu()
    imu.header.stamp = node.get_clock().now().to_msg()
    imu.header.frame_id = 'imu_link'
    imu.linear_acceleration.z = 9.80665
    return imu


def test_valid_imu_publishes_angles_and_monitor_commands(node):
    node.on_imu(imu_sample(node))
    node.tick()
    assert len(node.angles_pub.messages) == 1
    assert node.angles_pub.messages[0].name == node.policy.names
    np.testing.assert_allclose(node.angles_pub.messages[0].position,
                               np.clip(np.array(node.actions_pub.messages[0].data) * 0.2,
                                       *node.policy.target_limits), rtol=1e-6)
    assert all(mode == 2 for mode in node.command_pub.messages[0].mode)


def test_imu_timeout_latches_and_fresh_data_cannot_restart_output(node):
    node.on_imu(imu_sample(node))
    node.tick()
    node.imu = (time.monotonic() - 0.2, *node.imu[1:])
    node.tick()
    assert node.fault == 'IMU timeout'
    node.on_imu(imu_sample(node))
    node.tick()
    assert len(node.angles_pub.messages) == 1


def test_old_source_timestamp_is_rejected_despite_fresh_receipt(node):
    msg = imu_sample(node)
    msg.header.stamp.sec -= 1
    node.on_imu(msg)
    node.tick()
    assert node.rejected_imu_timestamps == 1
    assert node.imu is None
    assert not node.angles_pub.messages


def test_missing_or_invalid_feedback_stops_output(node):
    node.feedback_mode = 'joint_states'
    node.on_imu(imu_sample(node))
    node.tick()
    assert not node.angles_pub.messages
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = node.policy.names[::-1]
    msg.position = [0.01 * i for i in range(10)]
    msg.velocity = [0.0] * 10
    node.on_joints(msg)
    np.testing.assert_allclose(node.joints[2], msg.position[::-1], rtol=1e-6)
    node.tick()
    assert len(node.angles_pub.messages) == 1
    msg.velocity = []
    node.on_joints(msg)
    node.tick()
    assert node.fault is not None
    assert len(node.angles_pub.messages) == 1


def test_joint_timeout_and_estop_gate_output(node):
    node.feedback_mode = 'joint_states'
    node.on_imu(imu_sample(node))
    node.joints = (time.monotonic() - 0.2, node.ros_seconds(), np.zeros(10), np.zeros(10))
    node.tick()
    assert node.fault == 'JointState timeout'
    assert not node.command_pub.messages


def test_estop_latches(node):
    node.on_imu(imu_sample(node))
    node.on_estop(Bool(data=True))
    node.tick()
    assert node.fault == 'estop'
    assert not node.command_pub.messages


def test_unavailable_attitude_quaternion_is_rejected(node):
    node.orientation_source = 'message'
    msg = imu_sample(node)
    msg.orientation.w = 1.0
    msg.orientation_covariance[0] = -1.0
    node.on_imu(msg)
    node.tick()
    assert node.fault == 'IMU orientation is unavailable or invalid'
    assert not node.angles_pub.messages


def spike(node, accel=110.0):
    msg = imu_sample(node)
    msg.linear_acceleration.z = accel
    return msg


def test_acceleration_outlier_latches_by_default(node):
    assert node.imu_accel_outlier == 'latch'
    node.on_imu(spike(node))
    assert node.fault == 'Invalid IMU acceleration'


def test_skip_mode_drops_an_isolated_acceleration_spike(node):
    node.imu_accel_outlier = 'skip'
    node.on_imu(imu_sample(node))
    previous = node.imu
    node.on_imu(spike(node))
    # Complementary attitude uses acceleration, so the spike is dropped, not fused.
    assert node.fault is None and node.imu_accel_outliers == 1 and node.imu is previous
    node.tick()  # the last valid sample is still fresh
    assert len(node.angles_pub.messages) == 1
    node.on_imu(imu_sample(node))
    assert node.accel_outlier_since is None


def test_skip_mode_latches_persistent_acceleration_outliers(node):
    node.imu_accel_outlier = 'skip'
    node.on_imu(spike(node))
    node.accel_outlier_since = time.monotonic() - 0.2  # outliers for longer than input_timeout_s
    node.on_imu(spike(node))
    assert node.fault == 'Invalid IMU acceleration persisted'
