"""Registry/model consistency and calibrated command tests without CAN I/O."""
import copy
import hashlib
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

import yaml

from motor_control_hybrid.motor_configuration import MotorSettings, validate_registry
from motor_control_hybrid.python_can_node import PythonCanNode
from motor_control_hybrid.tracking_onnx import TrackingPolicy, validate_tracking_registry, apply_runtime_policy


REPO = Path(__file__).resolve().parents[3]
REGISTRY = REPO / 'humanoid_control/motor_control_hybrid/config/motors.yaml'
MODEL = REPO.parent / 'simulation/logs/legs_tracking/20260914_170827/20260914_170827.onnx'


class MotorConfigurationTest(unittest.TestCase):
    def test_simulation_registry_refuses_hardware(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        with self.assertRaisesRegex(ValueError, 'not hardware verified'):
            validate_registry(params)
        params['hardware_verified'] = True
        settings = validate_registry(params)
        self.assertEqual(len(settings), 10)

    def test_runtime_settings_match_source_by_name(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        policy = SimpleNamespace(names=list(reversed(params['motors'])))
        apply_runtime_policy(policy, params)
        self.assertEqual(policy.kp[0], 120)
        self.assertAlmostEqual(policy.offset[0], 0.4, places=6)
        self.assertEqual(policy.scale[0], 1)
        self.assertEqual(params['runtime_policy']['control_hz'], 50)

    def test_ankle_conversion_round_trip(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        settings = MotorSettings.from_config(params['motors']['left_ankle_joint'], params)
        for target in (-0.5, 0, 0.4):
            command = settings.command(dict(position=target, velocity=0, acceleration=0, torque=0, kp=120, kd=0.8))
            joint, _, _ = settings.feedback(command['position'], 0, 0)
            self.assertAlmostEqual(joint, target, places=6)

    def test_duplicate_bus_ids_and_conflicting_master_ids_rejected(self):
        cfg = {'motor_id': 12, 'model': 'rs-03', 'can_interface': 'can0', 'master_id': 255}
        params = {'motors': {'a': cfg, 'b': dict(cfg)}}
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            validate_registry(params)
        params['motors']['b'].update(motor_id=13, master_id=254)
        with self.assertRaisesRegex(ValueError, 'share a master_id'):
            validate_registry(params)

    def test_calibration_round_trip_and_limits(self):
        settings = MotorSettings.from_config(
            {'direction': -1, 'encoder_offset_rad': 0.4, 'min_position': -1,
             'max_position': 1, 'kp': 80, 'kd': 1, 'max_torque_nm': 5},
            {'MAX_VEL_RAD_S': 2})
        cmd = settings.command(dict(position=3, velocity=4, acceleration=0,
                                    torque=9, kp=80, kd=1))
        self.assertAlmostEqual(cmd['position'], -0.6)
        self.assertEqual((cmd['velocity'], cmd['torque']), (-2, -5))
        self.assertEqual(cmd['kp'], 80)
        q, dq, torque = settings.feedback(cmd['position'], cmd['velocity'], cmd['torque'])
        self.assertAlmostEqual(q, 1)
        self.assertEqual((dq, torque), (2, 5))
        with self.assertRaisesRegex(ValueError, 'Non-finite'):
            settings.feedback(float('nan'), 0, 0)

    def test_can_callback_uses_per_joint_gains(self):
        settings = MotorSettings.from_config({'kp': 80, 'kd': 1}, {})
        commands = []
        node = SimpleNamespace(estop_active=threading.Event(), motor_name_by_joint={'knee': 'motor_12'},
                               motor_settings={'knee': settings},
                               command_queue=SimpleNamespace(put_nowait=commands.append))
        msg = SimpleNamespace(joint_name=['knee'], mode=[2], position=[0.2], velocity=[],
                              acceleration=[], torque=[], kp=[], kd=[])
        PythonCanNode._cmd_callback(node, msg)
        self.assertEqual((commands[0]['kp'], commands[0]['kd']), (80, 1))
        msg.kp, msg.kd = [25], [0.5]
        PythonCanNode._cmd_callback(node, msg)
        self.assertEqual((commands[-1]['kp'], commands[-1]['kd']), (25, 0.5))

    def test_cached_feedback_expires_and_uses_registry_order(self):
        messages = []
        now = time.monotonic()
        node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
            state_lock=threading.Lock(), motor_name_by_joint={'a': 'motor_1', 'b': 'motor_2'},
            state_buffer={'b': (2., 0., 0., 28., now), 'a': (1., 0., 0., 28., now)},
            feedback_timeout_s=0.1, joint_state_pub=SimpleNamespace(publish=messages.append),
            _publish_motor_status=lambda: None)
        # JointState requires a Time object rather than None.
        from builtin_interfaces.msg import Time
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
        PythonCanNode._publish_states(node)
        self.assertEqual(messages[-1].name, ['a', 'b'])
        node.state_buffer['a'] = (1., 0., 0., 28., now - 1)
        PythonCanNode._publish_states(node)
        self.assertEqual(messages[-1].name, ['b'])
        node.state_buffer['b'] = (2., 0., 0., 28., now - 1)
        count = len(messages)
        PythonCanNode._publish_states(node)
        self.assertEqual(len(messages), count)

    def test_can_motion_sends_calibrated_limited_values(self):
        from motor_control_interfaces.msg import MotorCommand
        sent = []
        settings = MotorSettings.from_config(
            {'direction': -1, 'encoder_offset_rad': 0.4, 'min_position': -1,
             'max_position': 1, 'kp': 80, 'kd': 1, 'max_torque_nm': 5}, {})
        node = PythonCanNode.__new__(PythonCanNode)
        node.bus_by_joint = {'knee': SimpleNamespace(channel='can0', write_operation_frame=lambda *args: sent.append(args))}
        node.motor_name_by_joint = {'knee': 'motor_12'}
        node.motor_settings = {'knee': settings}
        node.bus_locks = {'can0': threading.Lock()}
        node.current_mode_by_joint = {'knee': 0}
        node.max_vel_rad_s = 4.5
        node.default_kp, node.default_kd = 10.0, 0.2
        node._try_read_feedback = lambda joint: None
        node._send_active_command(dict(joint='knee', mode=MotorCommand.MODE_MOTION,
                                      position=3, velocity=0.3, acceleration=0,
                                      torque=9, kp=80, kd=1))
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][0], 'motor_12')
        self.assertAlmostEqual(sent[0][1], -0.6)
        self.assertEqual(sent[0][2:], (80, 1, -0.3, -5))

    @unittest.skipUnless(MODEL.exists(), 'External tracking export unavailable')
    def test_registry_matches_model_and_detects_drift(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        policy = TrackingPolicy(MODEL)
        sha = hashlib.sha256(MODEL.read_bytes()).hexdigest()
        validate_tracking_registry(policy, params, sha)
        wrong = copy.deepcopy(params)
        wrong['model_contract']['joint_stiffness'][3] = 100
        with self.assertRaisesRegex(ValueError, 'joint_stiffness differs'):
            validate_tracking_registry(policy, wrong, sha)
        wrong = copy.deepcopy(params)
        wrong['model_contract']['joint_order'][0:2] = reversed(wrong['model_contract']['joint_order'][0:2])
        with self.assertRaisesRegex(ValueError, 'does not match'):
            validate_tracking_registry(policy, wrong, sha)


if __name__ == '__main__':
    unittest.main()
