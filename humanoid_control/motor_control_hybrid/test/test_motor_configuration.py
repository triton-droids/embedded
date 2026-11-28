"""Registry/model consistency and calibrated command tests without CAN I/O."""
import copy
import hashlib
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

import yaml

from motor_control_hybrid.motor_configuration import MotorSettings, validate_registry
from motor_control_hybrid.python_can_node import PythonCanNode


REPO = Path(__file__).resolve().parents[3]
REGISTRY = REPO / 'humanoid_control/motor_control_hybrid/config/motors.yaml'
MODEL = REPO.parent / 'simulation/logs/legs_tracking/20260914_170827/20260914_170827.onnx'
HAS_ONNX = importlib.util.find_spec('onnx') is not None and importlib.util.find_spec('onnxruntime') is not None


class MotorConfigurationTest(unittest.TestCase):
    def test_simulation_registry_refuses_hardware(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        with self.assertRaisesRegex(ValueError, 'not hardware verified'):
            validate_registry(params)
        params['hardware_verified'] = True
        settings = validate_registry(params)
        self.assertEqual(len(settings), 10)

    def test_runtime_settings_match_source_by_name(self):
        from motor_control_hybrid.tracking_onnx import apply_runtime_policy
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        names = list(reversed(params['motors']))
        policy = SimpleNamespace(names=names,
                                 kp=[params['motors'][n]['kp'] for n in names],
                                 kd=[params['motors'][n]['kd'] for n in names],
                                 offset=[0.] * 10, scale=[0.2] * 10)
        apply_runtime_policy(policy, params)
        self.assertEqual(policy.kp[0], 20)
        self.assertEqual(policy.kd[0], 1)
        self.assertEqual(policy.offset, [0.] * 10)
        self.assertEqual(policy.scale, [0.2] * 10)
        self.assertNotIn('runtime_policy', params)
        self.assertEqual(params['model_contract']['control_hz'], 50)

    def test_ankle_conversion_round_trip(self):
        params = yaml.safe_load(REGISTRY.read_text())['motor_control_node']['ros__parameters']
        settings = MotorSettings.from_config(params['motors']['left_ankle_joint'], params)
        for target in (-0.5, 0, 0.4):
            command = settings.command(dict(position=target, velocity=0, acceleration=0, torque=0, kp=120, kd=0.8))
            joint, _, _ = settings.feedback(command['position'], 0, 0)
            self.assertAlmostEqual(joint, target, places=6)

    def test_duplicate_bus_ids_and_conflicting_master_ids_rejected(self):
        cfg = {'motor_id': 12, 'model': 'rs-03', 'can_interface': 'can1', 'master_id': 255}
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
                               transport_fault=threading.Event(), _tx_wakeup=threading.Event(),
                               motor_settings={'knee': settings},
                               command_timeout_s=0.5,
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100 * 10**9)),
                               command_queue=SimpleNamespace(put_nowait=commands.extend))
        from builtin_interfaces.msg import Time
        msg = SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=100)), valid_until=Time(),
                              joint_name=['knee'], mode=[2], position=[0.2], velocity=[],
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
        node.bus_by_joint = {'knee': SimpleNamespace(channel='can1', write_operation_frame=lambda *args: sent.append(args))}
        node.motor_name_by_joint = {'knee': 'motor_12'}
        node.motor_settings = {'knee': settings}
        node.bus_locks = {'can1': threading.Lock()}
        node.current_mode_by_joint = {'knee': 0}
        node.estop_active = threading.Event()
        node.transport_fault = threading.Event()
        node.command_deadlines = {'knee': time.monotonic() + 0.5}
        node.inhibited_since = {}
        node.pending_disable = set()
        node.enable_pending = {}
        node._awaiting_enable_ack = threading.Event()
        node.active_commands = {}
        node.active_command_lock = threading.Lock()
        node.max_vel_rad_s = 4.5
        node.default_kp, node.default_kd = 10.0, 0.2
        node._try_read_feedback = lambda joint: None
        node.state_lock = threading.Lock()
        node.feedback_timeout_s = 0.1
        # Fresh raw feedback at the clamped target: the torque budget is spent
        # by the 5 N.m feedforward, so the target may not leave the measurement.
        node.raw_feedback = {'knee': (SimpleNamespace(position=-0.6), time.monotonic(), 0.0)}
        node._send_active_command(dict(joint='knee', mode=MotorCommand.MODE_MOTION,
                                      position=3, velocity=0.3, acceleration=0,
                                      torque=9, kp=80, kd=1))
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][0], 'motor_12')
        self.assertAlmostEqual(sent[0][1], -0.6)
        self.assertEqual(sent[0][2:], (80, 1, -0.3, -5))

    @unittest.skipUnless(MODEL.exists() and HAS_ONNX, 'External tracking export or ONNX dependencies unavailable')
    def test_registry_matches_model_and_detects_drift(self):
        from motor_control_hybrid.tracking_onnx import TrackingPolicy, validate_tracking_registry
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


class MotionSafetyLimitTest(unittest.TestCase):
    """Driver-side guards: gain ceilings, tracking-error clamp, jump detection."""

    def settings(self, **extra):
        cfg = {'model': 'rs-03', 'min_position': -1, 'max_position': 1, 'kp': 100, 'kd': 1,
               'max_torque_nm': 40}
        cfg.update(extra)
        return MotorSettings.from_config(cfg, {'MAX_POSITION_ERROR_RAD': 0.3})

    @staticmethod
    def cmd(**kw):
        base = dict(mode=2, position=0.0, velocity=0.0, acceleration=0.0, torque=0.0, kp=100, kd=1)
        base.update(kw)
        return base

    def test_target_is_kept_near_measured_position(self):
        out = self.settings().command(self.cmd(position=0.9), measured_motor=0.0)
        self.assertAlmostEqual(out['position'], 0.3)  # MAX_POSITION_ERROR_RAD

    def test_error_bound_is_not_a_torque_budget(self):
        # A torque-budget clamp made the simulated robot fall; only the fixed
        # MAX_POSITION_ERROR_RAD (0.3 here) applies, whatever kp and max torque are.
        out = self.settings().command(self.cmd(position=-0.9, kp=200, torque=10), measured_motor=0.0)
        self.assertEqual(out['kp'], 100)  # gain ceiling
        self.assertAlmostEqual(out['position'], -0.3)
        out = self.settings(max_torque_nm=5).command(self.cmd(position=-0.9), measured_motor=0.0)
        self.assertAlmostEqual(out['position'], -0.3)

    def test_position_frame_without_feedback_is_a_zero_gain_probe(self):
        out = self.settings().command(self.cmd(position=0.9, torque=5, velocity=1.0),
                                      measured_motor=None)
        self.assertEqual((out['kp'], out['kd'], out['torque'], out['velocity']), (0.0, 0.0, 0.0, 0.0))
        self.assertTrue(out['probe'])
        # Velocity mode has no position target and needs no measurement.
        self.settings().command(self.cmd(mode=0, velocity=1.0))

    def test_gain_ceilings(self):
        out = self.settings(max_kd=2).command(self.cmd(kp=500, kd=9), measured_motor=0.0)
        self.assertEqual((out['kp'], out['kd']), (100, 2))

    def test_feedback_jump_is_rejected(self):
        s = self.settings()  # rs-03 no-load 20.4 rad/s
        s.check_feedback_step((0.0, 1.0), (0.1, 1.002))  # 0.1 rad in 2 ms: below 0.05+0.061
        with self.assertRaises(ValueError):
            s.check_feedback_step((0.0, 1.0), (0.8, 1.002))  # re-zeroed 0.8 rad away
        s.check_feedback_step((0.0, 1.0), (0.8, 1.5))  # outside the comparison window

    def test_max_torque_cannot_exceed_model_peak(self):
        with self.assertRaises(ValueError):
            self.settings(model='rs-02', max_torque_nm=20)



class HardLimitTest(unittest.TestCase):
    def test_command_range_must_stay_inside_mechanical_range(self):
        cfg = {'model': 'rs-04', 'min_position': -2.0, 'max_position': 0.0}
        MotorSettings.from_config(dict(cfg, hard_min_position=-2.3, hard_max_position=2.3), {})
        MotorSettings.from_config(dict(cfg, hard_min_position=None, hard_max_position=None), {})
        with self.assertRaises(ValueError):
            MotorSettings.from_config(dict(cfg, hard_min_position=-1.9), {})

    def test_repository_registry_is_consistent(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'motors.yaml'
        params = yaml.safe_load(path.read_text())['motor_control_node']['ros__parameters']
        for name, cfg in params['motors'].items():
            settings = MotorSettings.from_config(cfg, params)
            self.assertIsNotNone(settings.max_torque, name)
            self.assertEqual(settings.max_position_error, 2.0, name)


class FirmwareParameterTest(unittest.TestCase):
    def test_can_timeout_seconds_become_manual_ticks(self):
        from motor_control_hybrid.motor_configuration import firmware_writes
        self.assertEqual(firmware_writes({}, {'FIRMWARE_CAN_TIMEOUT_S': 1.0}),
                         ((0x7028, 'u32', 'canTimeout', 20000),))  # 20000 = 1 s
        self.assertEqual(firmware_writes({'firmware_can_timeout_s': 0.75}, {}),
                         ((0x7028, 'u32', 'canTimeout', 15000),))
        self.assertEqual(firmware_writes({'firmware_can_timeout_s': None},
                                         {'FIRMWARE_CAN_TIMEOUT_S': 1.0}), ())
        for bad in (0, -1, 5.5, True, '1'):
            with self.assertRaises(ValueError):
                firmware_writes({'firmware_can_timeout_s': bad}, {})

    def test_enable_writes_and_requests_readback(self):
        calls = []
        node = PythonCanNode.__new__(PythonCanNode)
        node.motor_settings = {'knee': MotorSettings.from_config(
            {'model': 'rs-04', 'firmware_can_timeout_s': 1.0}, {})}
        node._write_parameter = lambda bus, joint, motor, ptype, value: calls.append(('w', ptype[0], value))
        bus = SimpleNamespace(read_parameter=lambda motor, index: calls.append(('r', index)))
        self.assertTrue(node._apply_firmware_parameters(bus, 'knee', 'motor_4'))
        self.assertEqual(calls, [('w', 0x7028, 20000), ('r', 0x7028)])

    def test_repository_registry_sets_the_motor_watchdog(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'motors.yaml'
        params = yaml.safe_load(path.read_text())['motor_control_node']['ros__parameters']
        for name, cfg in params['motors'].items():
            writes = MotorSettings.from_config(cfg, params).firmware_writes
            self.assertEqual([w[0] for w in writes], [0x7028], name)
