import json
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from chase_policy import ChaseTrackingPolicy, dynamic_reference, request_reason


def request(vx=0.2, wz=0.0, stamp=100.0, receipt=10.0, frame='base_link'):
    return receipt, stamp, frame, (vx, 0.0, 0.0, 0.0, 0.0, wz)


@pytest.mark.parametrize('command,now_ros,now_wall,sim,reason', [
    (None, 100, 10, None, 'waiting_for_fresh_simulator'),
    (None, 100, 10, 10, 'waiting_for_chase'),
    (request(), 100, 10, 10, None),
    (request(vx=0), 100, 10, 10, 'zero_request_target_lost_or_close'),
    (request(wz=0.3), 100, 10, 10, 'turn_required_not_implemented'),
    (request(vx=float('nan')), 100, 10, 10, 'invalid_command'),
    (request(frame='world'), 100, 10, 10, 'invalid_command'),
    (request(stamp=99.5), 100, 10, 10, 'stale_chase_command'),
    (request(stamp=100.1), 100, 10, 10, 'stale_chase_command'),
    (request(receipt=9.5), 100, 10, 10, 'stale_chase_command'),
    (request(vx=-0.1), 100, 10, 10, 'unsupported_command'),
    (request(vx=0.3), 100, 10, 10, 'unsupported_command'),
])
def test_request_contract(command, now_ros, now_wall, sim, reason):
    assert request_reason(command, now_ros, now_wall, sim) == reason


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(ChaseTrackingPolicy, 'run_worker', lambda self: None)
    rclpy.init(args=['--ros-args', '-p', 'calibration_seconds:=0.0',
                    '-p', 'wait_for_sim_release:=true', '-p', 'joint_feedback_mode:=joint_states'])
    n = ChaseTrackingPolicy()
    n.command_pub, n.angles_pub, n.actions_pub = Capture(), Capture(), Capture()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def fresh_inputs(node, vx=0.2, wz=0.0):
    node.on_sim_status(String(data=json.dumps(dict(
        base_held=True, sim_time_s=1.0, control_fault=None, estop_active=False))))
    command = TwistStamped()
    command.header.frame_id = 'base_link'
    command.header.stamp = node.get_clock().now().to_msg()
    command.twist.linear.x, command.twist.angular.z = vx, wz
    node.on_chase(command)
    imu = Imu()
    imu.header = command.header
    imu.header.frame_id = 'imu_link'
    imu.orientation.w = 1.0
    imu.linear_acceleration.z = 9.80665
    node.on_imu(imu)
    joints = JointState()
    joints.header = imu.header
    joints.name = node.policy.names
    joints.position = node.policy.offset.astype(float).tolist()
    joints.velocity = [0.0]*10
    node.on_joints(joints)


def test_no_chase_no_motor_messages(node):
    node.tick()
    assert not node.command_pub.messages and not node.armed


def test_real_policy_warps_from_chase_and_latches_zero(node):
    fresh_inputs(node)
    node.tick()
    assert node.command_pub.messages and node.armed
    assert node.requested_scale == pytest.approx(1.05)
    assert 1.0 < node.scale <= 1.0021
    np.testing.assert_array_equal(node.policy.reference_q[0], node.original_q[0])
    assert np.max(np.abs(node.policy.reference_q-node.original_q)) > 0
    count = len(node.command_pub.messages)
    fresh_inputs(node, vx=0.0)
    assert 'zero_request' in node.fault  # callback also rejects in-flight results
    node.tick()
    assert 'zero_request' in node.fault
    fresh_inputs(node)
    node.tick()
    assert len(node.command_pub.messages) == count


@pytest.mark.parametrize('mode', ['turn', 'command_timeout', 'time_limit'])
def test_invalidation_stops_refreshing_existing_motor_commands(node, mode):
    fresh_inputs(node)
    node.tick()
    count = len(node.command_pub.messages)
    fresh_inputs(node, wz=0.3 if mode == 'turn' else 0.0)
    if mode == 'command_timeout':
        node.chase_command = (time.monotonic()-0.3, *node.chase_command[1:])
    if mode == 'time_limit':
        node.reference_started = time.monotonic()-node.max_walk_s-1
    node.tick()
    assert node.fault and len(node.command_pub.messages) == count


def test_dynamic_scale_derivative_is_in_velocity(node):
    a, b = dynamic_reference(node.original_q, node.original_dq, node.policy.names, 1.0, 0.1)
    np.testing.assert_array_equal(a, node.original_q)
    np.testing.assert_array_equal(b[0], node.original_dq[0])
    i = node.policy.names.index('left_hip1_joint')
    assert b[100, i] == pytest.approx(node.original_dq[100, i]
                                      +0.1*(node.original_q[100, i]-node.original_q[0, i]))


def test_initial_calibration_does_not_arm_or_latch_on_transient_loss(node):
    node.calibrated = False
    node.calibration_seconds = 2.0
    fresh_inputs(node)
    node.tick()
    assert not node.armed and not node.command_pub.messages
    fresh_inputs(node, vx=0.0)
    node.tick()
    assert not node.fault
    node.calibrated = True
    fresh_inputs(node)
    node.tick()
    assert node.armed and node.command_pub.messages


def test_first_stop_reason_survives_following_estop_and_simulator_fault(node):
    from std_msgs.msg import Bool
    fresh_inputs(node)
    node.tick()
    fresh_inputs(node, wz=0.3)
    first = node.fault
    assert 'turn_required' in first
    node.on_estop(Bool(data=True))
    node.on_sim_status(String(data=json.dumps(dict(base_held=False, sim_time_s=3.0,
        control_fault='command expired', estop_active=True))))
    node.tick()
    assert node.fault == first
    assert node.chase_reason == 'turn_required_not_implemented'
