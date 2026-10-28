"""Exercise callback faults and output gating without any serial or motor I/O."""
from pathlib import Path
import time
from unittest.mock import Mock

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
    model = (Path(__file__).resolve().parents[1] /
             'models/20260914_170827/20260914_170827.onnx')
    if not model.exists():
        pytest.skip('LFS policy export absent')
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
    imu.orientation.w = 1.0
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


def test_default_quaternion_gravity_keeps_gyro_separate(node):
    assert node.orientation_source == 'message'
    assert node.filter is None
    msg = imu_sample(node)
    # A 30-degree sensor pitch determines gravity even if acceleration points
    # sideways due to motion; the raw gyro must not be replaced by orientation.
    msg.orientation.w = float(np.cos(np.pi / 12))
    msg.orientation.y = float(np.sin(np.pi / 12))
    msg.linear_acceleration.z = 0.0
    msg.linear_acceleration.y = 9.80665
    msg.angular_velocity.x = 0.12
    msg.angular_velocity.y = -0.23
    msg.angular_velocity.z = 0.34
    node.on_imu(msg)
    assert node.fault is None
    np.testing.assert_allclose(node.imu[2], [0.12, -0.23, 0.34])
    np.testing.assert_allclose(node.imu[3], [0.5, 0, -np.sqrt(3) / 2], atol=1e-7)


def test_hold_first_frame_keeps_reference_fixed_but_updates_actions(node):
    node.hold_first_frame = True
    node.started = time.monotonic() - 0.5
    node.policy.observation = Mock(wraps=node.policy.observation)
    node.on_imu(imu_sample(node))
    node.tick()
    assert node.frame == 0
    assert node.policy.observation.call_args.args[0] == 0
    node.started -= 1.0
    msg = imu_sample(node)
    msg.angular_velocity.x = 0.2
    node.on_imu(msg)
    node.tick()
    assert node.frame == 0
    assert node.policy.observation.call_args.args[0] == 0
    assert len(node.command_pub.messages) == 2
    assert node.policy.observation.call_args.args[1][0] == pytest.approx(0.2)


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


@pytest.mark.parametrize('stage', ['statistics', 'publish'])
def test_status_work_does_not_hold_feedback_state_lock(node, stage):
    import threading
    entered, release = threading.Event(), threading.Event()
    original = node.statistics

    def blocked(value):
        entered.set()
        assert release.wait(2.0)
        return original(value) if stage == 'statistics' else None

    if stage == 'statistics':
        node.statistics = blocked
        node.status_pub = Mock()
    else:
        node.status_pub = Mock()
        node.status_pub.publish.side_effect = blocked
    worker = threading.Thread(target=node.publish_status)
    worker.start()
    try:
        assert entered.wait(1.0)
        acquired = node.state_lock.acquire(timeout=0.05)
        assert acquired, 'status computation/publish blocked feedback state'
        if acquired:
            node.state_lock.release()
    finally:
        release.set()
        worker.join(timeout=2.0)
    assert not worker.is_alive()
