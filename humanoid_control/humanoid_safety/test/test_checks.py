import math

from humanoid_safety.checks import (
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuObservation,
    SafetySupervisor,
    quaternion_to_roll_pitch,
)


def observation(now, roll_deg=0.0, pitch_deg=0.0, valid=True):
    return ImuObservation(
        received_at=now,
        roll_rad=math.radians(roll_deg),
        pitch_rad=math.radians(pitch_deg),
        valid=valid,
    )


def build_supervisor(started_at=0.0):
    return SafetySupervisor([
        FallDetectionCheck(confirm_samples=3),
        ImuFreshnessCheck(
            timeout_s=0.25,
            startup_grace_s=1.0,
            started_at=started_at,
        ),
    ])


def test_fall_requires_consecutive_samples_and_latches():
    supervisor = build_supervisor()
    supervisor.observe_imu(observation(0.1, roll_deg=45.0))
    assert supervisor.evaluate(0.1) is False
    supervisor.observe_imu(observation(0.2, roll_deg=0.0))
    supervisor.evaluate(0.2)

    for now in (0.3, 0.4):
        supervisor.observe_imu(observation(now, pitch_deg=50.0))
        assert supervisor.evaluate(now) is False
    supervisor.observe_imu(observation(0.5, pitch_deg=50.0))
    assert supervisor.evaluate(0.5) is True
    assert supervisor.latched is True

    supervisor.observe_imu(observation(0.6))
    supervisor.evaluate(0.6)
    assert supervisor.latched is True
    success, _ = supervisor.try_reset(0.6)
    assert success is True
    assert supervisor.latched is False


def test_reset_is_blocked_while_robot_is_still_tilted():
    supervisor = build_supervisor()
    for now in (0.1, 0.2, 0.3):
        supervisor.observe_imu(observation(now, roll_deg=45.0))
        supervisor.evaluate(now)

    success, reason = supervisor.try_reset(0.3)
    assert success is False
    assert "reset blocked" in reason
    assert supervisor.latched is True


def test_imu_timeout_trips_after_startup_grace():
    supervisor = build_supervisor(started_at=10.0)
    assert supervisor.evaluate(10.5) is False
    assert supervisor.evaluate(11.1) is True
    assert "imu_freshness" in supervisor.reason


def test_timeout_reset_requires_fresh_imu():
    supervisor = build_supervisor()
    supervisor.evaluate(1.1)
    success, _ = supervisor.try_reset(1.1)
    assert success is False

    supervisor.observe_imu(observation(1.2))
    success, _ = supervisor.try_reset(1.2)
    assert success is True


def test_quaternion_conversion_and_invalid_input():
    half_angle = math.radians(45.0) / 2.0
    roll, pitch, valid = quaternion_to_roll_pitch(
        math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)
    )
    assert valid is True
    assert math.isclose(math.degrees(roll), 45.0)
    assert math.isclose(pitch, 0.0, abs_tol=1.0e-12)
    assert quaternion_to_roll_pitch(0.0, 0.0, 0.0, 0.0)[2] is False
