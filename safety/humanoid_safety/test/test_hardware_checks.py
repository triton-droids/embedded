import math

import pytest

from humanoid_safety.checks import (
    BatteryVoltageCheck,
    CanLinkCheck,
    CanObservation,
    FallDetectionCheck,
    ImuObservation,
    JointObservation,
    JointPositionLimitCheck,
    JointVelocityLimitCheck,
    MotorFaultCheck,
    MotorFeedbackCheck,
    MotorObservation,
    MotorTemperatureCheck,
    MotorTorqueCheck,
    PowerObservation,
    SafetySupervisor,
)


def joints(now, positions=None, velocities=None, efforts=None):
    return JointObservation(
        received_at=now,
        positions=positions or {},
        velocities=velocities or {},
        efforts=efforts or {},
    )


def test_observe_routes_each_type_only_to_its_hook():
    fall = FallDetectionCheck(confirm_samples=1)
    temperature = MotorTemperatureCheck(max_temp_c=80.0, reset_temp_c=70.0)
    supervisor = SafetySupervisor([fall, temperature])
    supervisor.observe(MotorObservation(received_at=0.0, temperatures_c={"knee": 90.0}))
    assert fall.evaluate(0.0).reason == "waiting for first IMU sample"
    supervisor.observe(ImuObservation(received_at=0.0, roll_rad=0.0, pitch_rad=0.0))
    assert supervisor.evaluate(0.0) is True
    assert "motor_temperature" in supervisor.reason
    assert "fall_detection" not in supervisor.reason


def test_unknown_observation_type_is_rejected():
    with pytest.raises(TypeError):
        SafetySupervisor([MotorFaultCheck()]).observe(object())


def test_motor_temperature_resets_only_after_cooling():
    check = MotorTemperatureCheck(max_temp_c=80.0, reset_temp_c=70.0)
    supervisor = SafetySupervisor([check])
    supervisor.observe(MotorObservation(received_at=0.0, temperatures_c={"knee": 85.0}))
    assert supervisor.evaluate(0.0) is True
    supervisor.observe(MotorObservation(received_at=0.1, temperatures_c={"knee": 75.0}))
    assert supervisor.try_reset(0.1)[0] is False
    supervisor.observe(MotorObservation(received_at=0.2, temperatures_c={"knee": 65.0}))
    assert supervisor.try_reset(0.2)[0] is True


def test_motor_temperature_treats_nan_as_hot():
    check = MotorTemperatureCheck()
    check.observe(MotorObservation(received_at=0.0, temperatures_c={"knee": math.nan}))
    assert check.evaluate(0.0).active is True


def test_motor_fault_bits_trip():
    check = MotorFaultCheck()
    check.observe(MotorObservation(received_at=0.0, fault_bits={"knee": 0}))
    assert check.evaluate(0.0).active is False
    check.observe(MotorObservation(received_at=0.1, fault_bits={"knee": 0x04}))
    result = check.evaluate(0.1)
    assert result.active is True and "knee=0x04" in result.reason


def test_motor_torque_ignores_short_spikes_and_uses_per_joint_limits():
    check = MotorTorqueCheck(default_limit_nm=50.0, limits_nm={"ankle": 10.0}, confirm_samples=3)
    check.observe(joints(0.0, efforts={"knee": 60.0}))
    check.observe(joints(0.1, efforts={"knee": 20.0}))
    assert check.evaluate(0.1).active is False
    for now in (0.2, 0.3, 0.4):
        check.observe(joints(now, efforts={"knee": 20.0, "ankle": -12.0}))
    result = check.evaluate(0.4)
    assert result.active is True and "ankle" in result.reason and "knee" not in result.reason


def test_motor_feedback_trips_on_missing_joint_after_grace():
    check = MotorFeedbackCheck(expected_joints=["knee", "ankle"], timeout_s=0.25,
                               startup_grace_s=1.0, started_at=0.0)
    assert check.evaluate(0.5).active is False
    assert check.evaluate(1.5).active is True
    check.observe(joints(2.0, positions={"knee": 0.0, "ankle": 0.0}))
    assert check.evaluate(2.1).active is False
    check.observe(joints(2.2, positions={"knee": 0.0}))
    result = check.evaluate(2.4)
    assert result.active is True and "ankle" in result.reason


def test_joint_position_limit_with_margin():
    check = JointPositionLimitCheck(limits_rad={"knee": (-1.0, 1.0)}, margin_rad=0.1)
    check.observe(joints(0.0, positions={"knee": 1.05, "untracked": 9.0}))
    result = check.evaluate(0.0)
    assert result.active is False and result.clearable is False
    check.observe(joints(0.1, positions={"knee": 1.2}))
    assert check.evaluate(0.1).active is True
    check.observe(joints(0.2, positions={"knee": 0.5}))
    result = check.evaluate(0.2)
    assert result.active is False and result.clearable is True


def test_joint_velocity_needs_consecutive_samples():
    check = JointVelocityLimitCheck(max_velocity_rad_s=5.0, confirm_samples=2)
    check.observe(joints(0.0, velocities={"knee": 6.0}))
    assert check.evaluate(0.0).active is False
    check.observe(joints(0.1, velocities={"knee": -6.0}))
    assert check.evaluate(0.1).active is True


def test_can_link_trips_on_fault_and_on_silence():
    check = CanLinkCheck(timeout_s=3.0, startup_grace_s=1.0, started_at=0.0)
    check.observe(CanObservation(received_at=1.0))
    assert check.evaluate(1.5).active is False
    assert check.evaluate(4.5).active is True
    check.observe(CanObservation(received_at=5.0, transport_fault=True))
    result = check.evaluate(5.0)
    assert result.active is True and result.clearable is False


def test_battery_voltage_sag_then_recovery():
    check = BatteryVoltageCheck(min_voltage_v=42.0, reset_voltage_v=44.0, confirm_samples=3)
    supervisor = SafetySupervisor([check])
    for now in (0.0, 0.1):
        supervisor.observe(PowerObservation(received_at=now, voltage_v=41.0))
    supervisor.observe(PowerObservation(received_at=0.2, voltage_v=45.0))
    assert supervisor.evaluate(0.2) is False
    for now in (0.3, 0.4, 0.5):
        supervisor.observe(PowerObservation(received_at=now, voltage_v=41.0))
    assert supervisor.evaluate(0.5) is True
    supervisor.observe(PowerObservation(received_at=0.6, voltage_v=43.0))
    assert supervisor.try_reset(0.6)[0] is False
    supervisor.observe(PowerObservation(received_at=0.7, voltage_v=44.5))
    assert supervisor.try_reset(0.7)[0] is True

