from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock
import sys
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'humanoid_control/wbc_control_node'))

from wbc_control_node.bridge import ContactSelector, fresh, ordered


def test_reorders_and_rejects_corrupt_feedback():
    np.testing.assert_array_equal(ordered(['b', 'a'], [2, 1], ['a', 'b']), [1, 2])
    for names, values in [(['a', 'a'], [1, 2]), (['a'], []), (['b'], [1]), (['a'], [np.nan])]:
        with pytest.raises(ValueError):
            ordered(names, values, ['a'])


def test_source_and_receipt_must_both_be_fresh():
    assert fresh(10**9, 5., 1_050_000_000, 5.05, .1)
    assert not fresh(10**9, 5., 1_200_000_000, 5.05, .1)
    assert not fresh(10**9, 5., 1_050_000_000, 5.2, .1)
    assert not fresh(2 * 10**9, 5., 1_050_000_000, 5.05, .1)


def test_contact_lift_and_touchdown_are_debounced():
    selector = ContactSelector(delay=.04)
    assert not selector.update([0, 0], [80, 80], [0, 0], 1.)
    assert selector.update([0, 0], [80, 80], [0, 0], 1.05) == {'left_foot', 'right_foot'}
    assert 'right_foot' in selector.update([0, 0], [80, 80], [0, .03], 1.06)
    assert selector.update([0, 0], [80, 80], [0, .03], 1.11) == {'left_foot'}
    assert selector.update([0, .03], [80, 0], [0, .03], 1.2) == {'left_foot'}
    selector.update([0, 0], [80, 80], [0, 0], 1.3)
    assert selector.update([0, 0], [80, 80], [0, 0], 1.35) == {'left_foot', 'right_foot'}


@pytest.fixture
def bridge_node():
    from motor_control_interfaces.msg import MotorCommand
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from rclpy.time import Time
    from wbc_control_node.controller import WBCController
    from wbc_control_node.node import WBCControlNode
    controller = WBCController(ROOT / 'humanoid_leg_description/urdf/human_offset_corrected.urdf')
    controller.set_torque_limits(np.full(10, 60.))
    now = time.time_ns()
    source = now - 1_000_000
    policy = MotorCommand()
    policy.header.stamp.sec, policy.header.stamp.nanosec = divmod(source, 10**9)
    policy.valid_until.sec, policy.valid_until.nanosec = divmod(now + 90_000_000, 10**9)
    policy.joint_name = list(reversed(controller.joint_names))
    policy.mode = [MotorCommand.MODE_MOTION]
    policy.position = [0.] * 10
    joints = JointState()
    joints.name = list(controller.joint_names)
    joints.position = [0.] * 10
    joints.velocity = [0.] * 10
    odom = Odometry()
    odom.header.frame_id = 'world'
    odom.child_frame_id = 'floating_base'
    odom.pose.pose.position.z = .71173
    odom.pose.pose.orientation.w = 1.
    feet = JointState()
    feet.header.frame_id = 'world'
    feet.name = ['left_foot', 'right_foot']
    feet.position = [0., 0.]
    feet.effort = [78.48, 78.48]
    node = SimpleNamespace(
        controller=controller, names=controller.joint_names, limits=np.full(10, 60.),
        lower=controller.model.lowerPositionLimit[7:], upper=controller.model.upperPositionLimit[7:],
        target_data=controller.model.createData(), selector=ContactSelector(),
        params={'input_timeout_s': .1, 'max_input_skew_s': .04, 'max_compute_s': 1.,
                'output_lifetime_s': .08, 'contact_mode': 'double_support', 'com_reference_mode': 'hold'},
        inputs={name: (msg, source, time.monotonic()) for name, msg in
                [('policy', policy), ('joints', joints), ('odom', odom), ('feet', feet)]},
        estop=False, fault='', started=False, state='waiting_for_inputs', solved_count=0,
        last_compute_ms=0., output=Mock(), get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=now)),
        get_logger=lambda: Mock(), last_policy_stamp=0,
    )
    node._fail = MethodType(WBCControlNode._fail, node)
    node._store = MethodType(WBCControlNode._store, node)
    return node


def test_bridge_solves_without_motor_pd_and_preserves_source_deadline(bridge_node):
    from wbc_control_node.bridge import stamp_ns
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    policy = node.inputs['policy'][0]
    WBCControlNode.tick(node)
    assert not node.fault and node.solved_count == 1
    command = node.output.publish.call_args.args[0]
    assert stamp_ns(command.header.stamp) == stamp_ns(policy.header.stamp)
    assert stamp_ns(command.valid_until) <= stamp_ns(policy.valid_until)
    assert command.joint_name == node.names
    assert all(value == 0 for value in command.kp + command.kd)
    assert np.isfinite(command.torque).all()
    assert max(abs(np.array(command.torque))) < 60


def test_expired_policy_disables_instead_of_republishing(bridge_node):
    from motor_control_interfaces.msg import MotorCommand
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.started = True
    node.inputs['policy'][0].valid_until.sec = 1
    node.inputs['policy'][0].valid_until.nanosec = 0
    WBCControlNode.tick(node)
    assert node.fault and node.solved_count == 0
    assert list(node.output.publish.call_args.args[0].mode) == [MotorCommand.MODE_DISABLE]


def test_corrupt_state_latches_fault(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.inputs['joints'][0].position[0] = float('nan')
    WBCControlNode.tick(node)
    assert node.fault and node.solved_count == 0


def test_estop_never_solves(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.estop = True
    WBCControlNode.tick(node)
    node.output.publish.assert_not_called()


def test_policy_torque_bridge_uses_policy_gains_and_keeps_backend_pd_zero(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.params['tracking_mode'] = 'policy_torque'
    node.controller.enable_policy_torque_tracking()
    policy = node.inputs['policy'][0]
    policy.position = [-.02 if 'knee' in name else 0. for name in policy.joint_name]
    policy.velocity = [0.] * 10
    policy.torque = [.1] * 10
    policy.kp = [20.] * 10
    policy.kd = [1.] * 10
    WBCControlNode.tick(node)
    assert not node.fault and node.solved_count == 1
    np.testing.assert_allclose(node.last_nominal_torque,
                               [-.3 if 'knee' in name else .1 for name in node.names])
    command = node.output.publish.call_args.args[0]
    assert all(value == 0 for value in command.kp + command.kd)
    assert np.isfinite(command.torque).all()


def test_impedance_output_matches_wbc_torque_without_double_counting(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.params['tracking_mode'] = 'policy_impedance'
    node.controller.enable_policy_torque_tracking()
    policy = node.inputs['policy'][0]
    policy.position = [-.02 if 'knee' in name else 0. for name in policy.joint_name]
    policy.velocity = [.01] * 10
    policy.torque = [.1] * 10
    policy.kp = [20.] * 10
    policy.kd = [1.] * 10
    WBCControlNode.tick(node)
    assert not node.fault and node.solved_count == 1
    command = node.output.publish.call_args.args[0]
    q = np.array(node.inputs['joints'][0].position)
    v = np.array(node.inputs['joints'][0].velocity)
    reconstructed = (np.array(command.kp) * (np.array(command.position) - q)
                     + np.array(command.kd) * (np.array(command.velocity) - v)
                     + np.array(command.torque))
    np.testing.assert_allclose(reconstructed, node.last_torque, atol=1e-9)
    assert all(x == 20. for x in command.kp)
    assert all(x == 1. for x in command.kd)


def test_impedance_saturation_preserves_the_trained_fast_pd_loop(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.params['tracking_mode'] = 'policy_impedance'
    node.controller.enable_policy_torque_tracking(max_correction=.02)
    policy = node.inputs['policy'][0]
    policy.position = [-.2 if 'knee' in name else 0. for name in policy.joint_name]
    policy.velocity = [0.] * 10
    policy.torque = [.1] * 10
    policy.kp = [1000.] * 10
    policy.kd = [1.] * 10
    WBCControlNode.tick(node)
    assert not node.fault and node.solved_count == 1
    command = node.output.publish.call_args.args[0]
    assert np.max(np.abs(np.array(command.torque) - .1)) <= .02 + 1e-5
    actual = np.clip(np.array(command.kp) * np.array(command.position)
                     + np.array(command.torque), -node.limits, node.limits)
    assert np.max(np.abs(actual - node.last_torque)) <= .02 + 1e-5


def test_gantry_arming_does_not_move_joints_before_reference_start(bridge_node):
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.base_held = True
    WBCControlNode.tick(node)
    assert not node.fault and node.solved_count == 1
    command = node.output.publish.call_args.args[0]
    assert all(x == 0 for x in command.kp + command.kd + command.torque)
    assert node.state == 'arming_on_gantry'


def test_arming_waits_for_fresh_inputs_but_release_activates_timeout_latch(bridge_node):
    from std_msgs.msg import String
    from wbc_control_node.node import WBCControlNode
    node = bridge_node
    node.base_held = True
    WBCControlNode.tick(node)
    assert not node.started
    node.inputs = {k: (msg, stamp - 200_000_000, received)
                   for k, (msg, stamp, received) in node.inputs.items()}
    WBCControlNode.tick(node)
    assert not node.fault
    WBCControlNode._sim_status(node, String(data='{"base_held": false}'))
    WBCControlNode.tick(node)
    assert node.fault == 'Input source/receipt timeout'
