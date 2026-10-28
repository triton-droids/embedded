"""Physics checks on the composed world, using the robot files from the source tree."""
from pathlib import Path

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip('mujoco')

from humanoid_sim import model_builder as mb  # noqa: E402
from humanoid_sim.motor_model import MODE_ENABLE, MODE_MOTION, MotorBank, limits_from_registry  # noqa: E402
from humanoid_sim.world import HeadServoConfig, SimWorld  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
HYBRID = REPO / 'humanoid_control' / 'motor_control_hybrid'


def make_world(foot_clearance_m=0.05, settle_s=0.0):
    registry = yaml.safe_load((HYBRID / 'config' / 'motors.yaml').read_text())
    limits = limits_from_registry(registry['motor_control_node']['ros__parameters'])
    model = mb.build_model(mb.WorldOptions(
        robot_xml=str(HYBRID / 'models' / '20260914_170827' / 'chrobot_16kg_candidate.xml'),
        mesh_dir=str(REPO / 'humanoid_leg_description' / 'meshes' / 'robot_meshes'),
        joint_torque_limits={name: lim.max_torque for name, lim in limits.items()},
        foot_clearance_m=foot_clearance_m))
    return SimWorld(model, MotorBank(limits), head=HeadServoConfig(), base_held=True,
                    settle_s=settle_s)


@pytest.fixture
def world():
    return make_world()


def run(world, seconds, start=0.0):
    steps = int(seconds / world.model.opt.timestep)
    for i in range(steps):
        world.step(start + i * world.model.opt.timestep)


def command_all(world, mode, positions=None, t=0.0):
    names = world.joint_names
    positions = positions if positions is not None else [0.0] * len(names)
    world.motors.submit(names, [mode], positions, [], [], [], [], stamp_s=100.0,
                        valid_until_s=0.0, now_ros_s=100.0, now_mono_s=t)


def test_feet_start_above_floor(world):
    for site in mb.FOOT_SITES:
        assert world.data.site(site).xpos[2] == pytest.approx(0.05, abs=1e-6)


def test_settled_gantry_passes_policy_calibration_checks():
    world = make_world(settle_s=2.0)
    gyros, accels = [], []
    for i in range(400):  # Two seconds, the policy's calibration window.
        world.step(i * world.model.opt.timestep)
        sample = world.body_imu()
        gyros.append(sample.gyro)
        accels.append(np.linalg.norm(sample.accel))
    # Thresholds from tracking_policy_node's startup calibration.
    assert np.max(np.std(gyros, axis=0)) < 0.03
    assert 8.8 < min(accels) and max(accels) < 10.8
    imu = world.body_imu()
    assert imu.accel == pytest.approx([0.0, 0.0, 9.81], abs=0.05)
    assert imu.quat_wxyz == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=0.01)


def test_motion_command_tracks_target():
    world = make_world(foot_clearance_m=0.3)  # Swing the foot well clear of the floor.
    knee = world.joint_names.index('left_knee_joint')
    target = [0.0] * len(world.joint_names)
    target[knee] = -0.6
    command_all(world, MODE_ENABLE)
    command_all(world, MODE_MOTION, target)
    run(world, 0.4)  # Within the 0.5 s command lifetime.
    sample = world.joint_snapshot()
    # Kp=80 against gravity leaves a small steady-state error.
    assert sample.position[knee] == pytest.approx(-0.6, abs=0.05)


def test_unenabled_motors_are_limp(world):
    run(world, 1.0)
    assert all(abs(x) < 1e-9 for x in world.joint_snapshot().effort)


def test_released_base_falls_without_motors(world):
    world.set_base_held(False)
    run(world, 2.0)
    pos, _, _, _ = world.base_state()
    assert pos[2] < 0.4  # Collapsed onto the floor.


def test_head_servo_tracks_and_releases(world):
    world.set_head_target(0.5)
    run(world, 0.5)
    assert world.head_angle() == pytest.approx(0.5, abs=0.02)
    world.set_head_target(None)
    run(world, 0.1, start=0.5)
    assert world.data.actuator(mb.HEAD_ACTUATOR).force[0] == 0.0


def test_release_readiness_requires_motion_and_remaining_budget(world):
    assert not world.motors.motion_ready(0.0)
    command_all(world, MODE_ENABLE)
    assert not world.motors.motion_ready(0.0)
    command_all(world, MODE_MOTION)
    assert world.motors.motion_ready(0.0, 0.04)
    assert not world.motors.motion_ready(0.48, 0.04)
    world.motors.disable(world.joint_names[0])
    assert not world.motors.motion_ready(0.0)


def test_control_expiry_pauses_physics_without_rearming(world):
    world.pause_on_control_fault = True
    command_all(world, MODE_ENABLE)
    command_all(world, MODE_MOTION)
    world.set_base_held(False)
    world.step(0.0)
    position = world.data.qpos.copy()
    sim_time = world.data.time
    world.step(0.51)  # The last motion command has expired.
    assert world.control_fault is not None
    assert world.data.time == sim_time
    np.testing.assert_array_equal(world.data.qpos, position)
    assert not any(world.motors.is_enabled(j) for j in world.joint_names)
    command_all(world, MODE_ENABLE, t=0.52)
    command_all(world, MODE_MOTION, t=0.52)
    world.step(0.53)
    assert world.data.time == sim_time  # New traffic cannot clear the pause.
    assert not any(world.motors.is_enabled(j) for j in world.joint_names)
    world.reset(base_held=True)
    assert world.control_fault is None
    assert world.base_held()


def test_rejected_packet_does_not_replace_still_valid_motion(world):
    world.pause_on_control_fault = True
    command_all(world, MODE_ENABLE)
    command_all(world, MODE_MOTION)
    world.set_base_held(False)
    rejected = world.motors.submit(
        world.joint_names, [MODE_MOTION], [], [], [], [], [],
        stamp_s=99.0, valid_until_s=99.1, now_ros_s=100.0, now_mono_s=0.03)
    assert rejected == world.joint_names
    world.step(0.03)
    assert world.control_fault is None
    assert world.data.time > 0
