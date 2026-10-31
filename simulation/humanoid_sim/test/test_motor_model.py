"""Command admission rules of the simulated motor bank (no ROS, no MuJoCo)."""
import math

import pytest

from humanoid_sim.motor_model import (
    MODE_DISABLE, MODE_ENABLE, MODE_MOTION, MODE_POSITION, MODE_VELOCITY,
    JointLimits, MotorBank, limits_from_registry)

NOW_ROS = 1000.0
NOW_MONO = 50.0


def make_bank(timeout=0.5):
    limits = {'knee': JointLimits(lower=-2.0, upper=0.0, kp=80.0, kd=1.0,
                                  max_torque=120.0, max_velocity=4.5)}
    return MotorBank(limits, command_timeout_s=timeout)


def submit(bank, mode, *, position=(), velocity=(), torque=(), kp=(), kd=(),
           stamp=NOW_ROS, valid_until=0.0, now_ros=NOW_ROS, now_mono=NOW_MONO):
    return bank.submit(['knee'], [mode], list(position), list(velocity), list(torque),
                       list(kp), list(kd), stamp_s=stamp, valid_until_s=valid_until,
                       now_ros_s=now_ros, now_mono_s=now_mono)


def test_motion_requires_enable():
    bank = make_bank()
    submit(bank, MODE_MOTION, position=[-0.5], kp=[80.0], kd=[1.0])
    assert not bank.is_enabled('knee')
    assert bank.setpoint('knee', 0.0, 0.005).kp == 0.0


def test_torque_only_readiness_is_opt_in_and_still_expires():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_MOTION, torque=[1.], kp=[0.], kd=[0.])
    assert not bank.motion_ready(NOW_MONO)
    bank.allow_torque_only_motion = True
    assert bank.motion_ready(NOW_MONO)
    assert not bank.motion_ready(NOW_MONO + .6)
    bank.set_estop(True)
    assert not bank.motion_ready(NOW_MONO)


def test_enable_is_idle_until_motion():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    assert bank.is_enabled('knee')
    assert bank.setpoint('knee', -0.3, 0.005).kp == 0.0
    submit(bank, MODE_MOTION, position=[-0.5], velocity=[0.1], torque=[2.0], kp=[80.0], kd=[1.0])
    sp = bank.setpoint('knee', -0.3, 0.005)
    assert (sp.kp, sp.kd, sp.q_ref, sp.dq_ref, sp.tau_ff) == (80.0, 1.0, -0.5, 0.1, 2.0)


def test_motion_target_is_clipped_to_registry_limits():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_MOTION, position=[0.7])
    sp = bank.setpoint('knee', 0.0, 0.005)
    assert sp.q_ref == 0.0
    assert (sp.kp, sp.kd) == (80.0, 1.0)  # Missing gains fall back to the registry.


@pytest.mark.parametrize('stamp, valid_until', [
    (0.0, 0.0),                    # unset stamp
    (NOW_ROS - 0.6, 0.0),          # older than the command lifetime
    (NOW_ROS + 0.05, 0.0),         # from the future
    (NOW_ROS, NOW_ROS - 0.01),     # already expired
])
def test_invalid_timestamps_are_rejected(stamp, valid_until):
    bank = make_bank()
    assert submit(bank, MODE_ENABLE, stamp=stamp, valid_until=valid_until) == ['knee']
    assert not bank.is_enabled('knee')


def test_non_finite_values_are_rejected():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    assert submit(bank, MODE_MOTION, position=[math.nan]) == ['knee']


def test_disable_ignores_timestamp():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_DISABLE, stamp=0.0)
    assert not bank.is_enabled('knee')


def test_timeout_inhibits_until_explicit_enable():
    bank = make_bank(timeout=0.5)
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_MOTION, position=[-0.5])
    bank.expire(NOW_MONO + 0.49)
    assert bank.is_enabled('knee')
    bank.expire(NOW_MONO + 0.51)
    assert not bank.is_enabled('knee')
    later_ros, later_mono = NOW_ROS + 1.0, NOW_MONO + 1.0
    submit(bank, MODE_MOTION, position=[-0.5], stamp=later_ros, now_ros=later_ros, now_mono=later_mono)
    assert not bank.is_enabled('knee')  # Motion traffic cannot renew a timed-out joint.
    submit(bank, MODE_ENABLE, stamp=later_ros, now_ros=later_ros, now_mono=later_mono)
    assert bank.is_enabled('knee')


def test_source_age_counts_against_deadline():
    bank = make_bank(timeout=0.5)
    submit(bank, MODE_ENABLE, stamp=NOW_ROS - 0.4)
    bank.expire(NOW_MONO + 0.11)
    assert not bank.is_enabled('knee')


def test_estop_disables_and_blocks_until_cleared():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    bank.set_estop(True)
    assert not bank.is_enabled('knee')
    submit(bank, MODE_ENABLE)
    assert not bank.is_enabled('knee')
    bank.set_estop(False)
    assert not bank.is_enabled('knee')  # Reset does not re-enable.
    submit(bank, MODE_ENABLE)
    assert bank.is_enabled('knee')


def test_position_mode_ramps_at_requested_speed():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    bank.seed_reference('knee', 0.0)
    submit(bank, MODE_POSITION, position=[-1.0], velocity=[1.0])
    refs = [bank.setpoint('knee', 0.0, 0.1).q_ref for _ in range(3)]
    assert refs == pytest.approx([-0.1, -0.2, -0.3])


def test_velocity_mode_uses_damping_only():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_VELOCITY, velocity=[9.0], kd=[0.0])
    sp = bank.setpoint('knee', -0.2, 0.005)
    assert sp.kp == 0.0
    assert sp.kd == bank.velocity_mode_kd
    assert sp.dq_ref == 4.5  # Clipped to max velocity.


def test_registry_torque_defaults_by_model():
    limits = limits_from_registry({'motors': {
        'a': {'model': 'rs-04', 'max_torque_nm': None},
        'b': {'model': 'rs-03', 'max_torque_nm': 30.0},
    }})
    assert limits['a'].max_torque == 120.0
    assert limits['b'].max_torque == 30.0


def test_readiness_diagnostic_distinguishes_expiry_from_disable():
    bank = make_bank()
    submit(bank, MODE_ENABLE)
    submit(bank, MODE_MOTION, position=[-.1], kp=[20.])
    assert bank.readiness_failure(NOW_MONO) == 'ready'
    bank.expire(NOW_MONO + .6)
    assert bank.readiness_failure(NOW_MONO + .6) == 'command expired for knee'
    submit(bank, MODE_ENABLE, now_mono=NOW_MONO + .6)
    submit(bank, MODE_DISABLE, now_mono=NOW_MONO + .6)
    assert bank.readiness_failure(NOW_MONO + .6) == 'disabled joint knee'
