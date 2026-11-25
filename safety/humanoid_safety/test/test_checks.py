import math

from humanoid_safety.checks import (
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuObservation,
    ImuStuckCheck,
    SafetySupervisor,
    quaternion_to_roll_pitch,
)


def observation(now, roll_deg=0.0, pitch_deg=0.0, valid=True,
                gyro=None, accel=None):
    return ImuObservation(
        received_at=now,
        roll_rad=math.radians(roll_deg),
        pitch_rad=math.radians(pitch_deg),
        valid=valid,
        angular_velocity=gyro,
        linear_acceleration=accel,
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


STILL = (0.0, 0.0, 0.0)
SPINNING = (3.0, 0.0, 0.0)  # rad/s, a real fall.


def test_tilt_jump_without_gyro_is_ignored_as_glitch():
    check = FallDetectionCheck(confirm_samples=3)
    check.observe_imu(observation(0.0, gyro=STILL))
    for i in range(5):
        check.observe_imu(observation(0.01 * (i + 1), roll_deg=60.0, gyro=STILL))
    result = check.evaluate(0.05)
    assert result.active is False
    assert "ignored_glitches=5" in result.reason

    check.observe_imu(observation(0.06, gyro=STILL))
    assert "ignored_glitches" not in check.evaluate(0.06).reason


def test_tilt_jump_with_gyro_still_counts_as_fall():
    check = FallDetectionCheck(confirm_samples=3)
    check.observe_imu(observation(0.0, gyro=STILL))
    check.observe_imu(observation(0.01, roll_deg=60.0, gyro=SPINNING))
    # Robot now lies still on the ground; small changes are not jumps.
    check.observe_imu(observation(0.02, roll_deg=61.0, gyro=STILL))
    check.observe_imu(observation(0.03, roll_deg=61.0, gyro=STILL))
    assert check.evaluate(0.03).active is True


def test_persistent_tilt_is_trusted_after_glitch_limit():
    check = FallDetectionCheck(confirm_samples=3, glitch_max_samples=4)
    check.observe_imu(observation(0.0, gyro=STILL))
    for i in range(4):
        check.observe_imu(observation(0.01 * (i + 1), pitch_deg=80.0, gyro=STILL))
        assert check.evaluate(0.0).active is False
    for i in range(3):
        check.observe_imu(observation(0.1 + 0.01 * i, pitch_deg=80.0, gyro=STILL))
    assert check.evaluate(0.2).active is True


def test_glitch_check_skipped_without_gyro_data():
    check = FallDetectionCheck(confirm_samples=1)
    check.observe_imu(observation(0.0))
    check.observe_imu(observation(0.01, roll_deg=60.0))
    assert check.evaluate(0.01).active is True


def test_glitch_check_can_be_disabled():
    check = FallDetectionCheck(confirm_samples=1, glitch_jump_deg=0.0)
    check.observe_imu(observation(0.0, gyro=STILL))
    check.observe_imu(observation(0.01, roll_deg=60.0, gyro=STILL))
    assert check.evaluate(0.01).active is True


def test_stuck_imu_trips_after_identical_samples_and_latches():
    supervisor = SafetySupervisor([ImuStuckCheck(stuck_samples=5)])
    for i in range(4):
        supervisor.observe_imu(
            observation(0.01 * i, roll_deg=1.0, gyro=(0.01, 0.0, 0.0),
                        accel=(0.0, 0.0, 9.81)))
        assert supervisor.evaluate(0.01 * i) is False
    supervisor.observe_imu(
        observation(0.05, roll_deg=1.0, gyro=(0.01, 0.0, 0.0),
                    accel=(0.0, 0.0, 9.81)))
    assert supervisor.evaluate(0.05) is True
    assert "imu_stuck" in supervisor.reason

    success, reason = supervisor.try_reset(0.05)
    assert success is False and "imu_stuck" in reason
    supervisor.observe_imu(
        observation(0.06, roll_deg=1.0, gyro=(0.02, 0.0, 0.0),
                    accel=(0.0, 0.0, 9.81)))
    success, _ = supervisor.try_reset(0.06)
    assert success is True


def test_noisy_imu_is_not_stuck():
    check = ImuStuckCheck(stuck_samples=3)
    assert check.evaluate(0.0).active is False
    for i in range(20):
        check.observe_imu(observation(0.01 * i, accel=(0.0, 0.0, 9.81 + i * 1e-4)))
    assert check.evaluate(0.2).active is False


def test_stuck_check_can_be_disabled():
    check = ImuStuckCheck(stuck_samples=2, enabled=False)
    for i in range(5):
        check.observe_imu(observation(0.01 * i))
    assert check.evaluate(0.05).active is False
