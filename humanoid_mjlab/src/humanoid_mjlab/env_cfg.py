"""Flat-ground forward-walking task for the humanoid.

Built from mjlab's velocity task (make_velocity_env_cfg). Only what differs for
this robot is set here.
"""

from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import (
    ContactMatch,
    ContactSensorCfg,
    ObjRef,
    RingPatternCfg,
)
from mjlab.tasks.velocity.mdp import (
    UniformVelocityCommand,
    UniformVelocityCommandCfg,
    feet_air_time,
)
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from humanoid_mjlab.crouch import joint_target_bounds, walking_robot_cfg

FORWARD_SPEED = 0.5  # m/s, root frame +X
NAV_YAW_RATE = 0.6  # rad/s, positive turns left
FOOT_CLEARANCE = 0.10  # m, mjlab G1 clearance and swing-height target
MIN_TORSO_HEIGHT = 0.50  # m; normal crouch holds the torso origin near 0.66 m
FOOT_SITES = ("left_foot", "right_foot")

# Radians per unit policy action, relative to the default standing pose.
# These are initial tuning values, not joint limits. The smaller ankle scale
# applies to the motor crank; foot and rod angles follow the passive linkage.
ACTION_SCALE = {
    ".*_hip1_joint": 0.25,
    ".*_hip2_joint": 0.15,
    ".*_thigh_joint": 0.15,
    ".*_knee_joint": 0.35,
    ".*_ankle_crank_joint": 0.10,
    ".*_shoulder_pitch_joint": 0.20,
    ".*_shoulder_roll_joint": 0.15,
    ".*_shoulder_yaw_joint": 0.15,
    ".*_elbow_joint": 0.20,
}

# Pose reward: how far (rad) each joint may move from the standing pose. Larger is
# looser. Walking values of mjlab's G1 task.
POSE_STD = {
    ".*_hip1_joint": 0.3,
    ".*_hip2_joint": 0.15,
    ".*_thigh_joint": 0.15,
    ".*_knee_joint": 0.35,
    ".*_ankle_crank_joint": 0.25,
    ".*_shoulder_pitch_joint": 0.15,
    ".*_shoulder_roll_joint": 0.15,
    ".*_shoulder_yaw_joint": 0.1,
    ".*_elbow_joint": 0.15,
}

POSE_STD_RUNNING = {
    ".*_hip1_joint": 0.5,
    ".*_hip2_joint": 0.2,
    ".*_thigh_joint": 0.2,
    ".*_knee_joint": 0.6,
    ".*_ankle_crank_joint": 0.35,
    ".*_shoulder_pitch_joint": 0.5,
    ".*_shoulder_roll_joint": 0.2,
    ".*_shoulder_yaw_joint": 0.15,
    ".*_elbow_joint": 0.35,
}


class FixedVelocityCommand(UniformVelocityCommand):
    def create_gui(self, *args, **kwargs) -> None:
        """No joystick: mjlab's viewer sliders crash when a command range is zero."""


@dataclass(kw_only=True)
class FixedVelocityCommandCfg(UniformVelocityCommandCfg):
    def build(self, env) -> FixedVelocityCommand:
        return FixedVelocityCommand(self, env)


class NavigationVelocityCommand(FixedVelocityCommand):
    def _resample_command(self, env_ids: torch.Tensor) -> None:
        super()._resample_command(env_ids)
        # Include zero-forward turns and exact straight walking in the training mix.
        pivot = torch.rand(len(env_ids), device=self.device) < self.cfg.pivot_fraction
        pivot &= ~(self.is_forward_env[env_ids] | self.is_standing_env[env_ids])
        self.vel_command_b[env_ids[pivot], 0] = 0.0


@dataclass(kw_only=True)
class NavigationVelocityCommandCfg(FixedVelocityCommandCfg):
    pivot_fraction: float = 0.2

    def build(self, env) -> NavigationVelocityCommand:
        return NavigationVelocityCommand(self, env)


def track_planar_velocity(env, std: float, command_name: str):
    command = env.command_manager.get_command(command_name)
    actual = env.scene["robot"].data.root_link_lin_vel_b[:, :2]
    return torch.exp(-(command[:, :2] - actual).square().sum(dim=1) / std**2)


def track_yaw_rate(env, std: float, command_name: str):
    command = env.command_manager.get_command(command_name)
    actual = env.scene["robot"].data.root_link_ang_vel_b[:, 2]
    return torch.exp(-(command[:, 2] - actual).square() / std**2)


def motor_joints() -> SceneEntityCfg:
    """The 18 motor joints, without the four passive ankle joints."""
    return SceneEntityCfg("robot", joint_names=tuple(ACTION_SCALE))


def single_support_air_time(
    env,
    sensor_name,
    threshold_min=0.05,
    threshold_max=0.5,
    command_name=None,
    command_threshold=0.5,
):
    """Reward a timed foot swing only while the other foot supports the robot."""
    reward = feet_air_time(
        env,
        sensor_name=sensor_name,
        threshold_min=threshold_min,
        threshold_max=threshold_max,
        command_name=command_name,
        command_threshold=command_threshold,
    )
    contact = env.scene[sensor_name].data.found
    assert contact is not None
    exactly_one_grounded = (contact > 0).sum(dim=1) == 1
    return reward * exactly_one_grounded


def make_env_cfg(play: bool = False, *, navigation: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()

    # Robot and flat ground.
    cfg.scene.num_envs = 4096
    cfg.scene.entities = {"robot": walking_robot_cfg()}
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None

    # 2 ms physics, 50 Hz policy.
    cfg.sim.mujoco.timestep = 0.002
    cfg.decimation = 10
    cfg.sim.njmax = 300  # constraints per robot, as mjlab's flat G1 task
    cfg.sim.nconmax = None

    # Actions: position targets for the 18 motors.
    cfg.actions["joint_pos"].actuator_names = tuple(ACTION_SCALE)
    cfg.actions["joint_pos"].scale = ACTION_SCALE.copy()
    cfg.actions["joint_pos"].clip = joint_target_bounds(tuple(ACTION_SCALE))

    # Command: always walk straight forward. The heading, world, standing and forward
    # command modes are off.
    cfg.commands["twist"] = FixedVelocityCommandCfg(
        entity_name="robot",
        resampling_time_range=(3.0, 8.0),
        heading_command=False,
        rel_heading_envs=0.0,
        rel_world_envs=0.0,
        rel_standing_envs=0.0,
        rel_forward_envs=0.0,
        debug_vis=True,
        ranges=UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(FORWARD_SPEED, FORWARD_SPEED),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
        ),
    )

    if navigation:
        cfg.commands["twist"] = NavigationVelocityCommandCfg(
            entity_name="robot",
            resampling_time_range=(1.0, 3.0),
            heading_command=False,
            rel_heading_envs=0.0,
            rel_world_envs=0.0,
            rel_standing_envs=0.15,
            rel_forward_envs=0.25,
            ranges=UniformVelocityCommandCfg.Ranges(
                lin_vel_x=(0.0, FORWARD_SPEED),
                lin_vel_y=(0.0, 0.0),
                ang_vel_z=(-NAV_YAW_RATE, NAV_YAW_RATE),
            ),
        )

    # Foot sensors. The terrain scan is not needed on flat ground.
    foot_height = next(s for s in cfg.scene.sensors if s.name == "foot_height_scan")
    foot_height.frame = tuple(
        ObjRef(type="site", name=name, entity="robot") for name in FOOT_SITES
    )
    foot_height.pattern = RingPatternCfg.single_ring(radius=0.03, num_samples=6)
    foot_contact = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(
            mode="body", pattern=r"^(left|right)_foot_link$", entity="robot"
        ),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="netforce",
        num_slots=1,
        track_air_time=True,
    )
    cfg.scene.sensors = (foot_height, foot_contact)

    # Observations. The robot model has no IMU sensors, so read the root velocity.
    for group in cfg.observations.values():
        group.terms["base_lin_vel"] = ObservationTermCfg(func=envs_mdp.base_lin_vel)
        group.terms["base_ang_vel"] = ObservationTermCfg(func=envs_mdp.base_ang_vel)
        group.terms["joint_pos"].params["asset_cfg"] = motor_joints()
        group.terms["joint_vel"].params["asset_cfg"] = motor_joints()
        del group.terms["height_scan"]
    cfg.observations["actor"].enable_corruption = False  # no observation noise yet

    # G1 reward baseline. Tracking, pose, and action-rate weights/stds inherit
    # mjlab's defaults; adapt entity selections to this robot's joint names.
    rewards = cfg.rewards
    rewards["upright"].params["asset_cfg"].body_names = ("torso_link",)
    rewards["body_ang_vel"].params["asset_cfg"].body_names = ("torso_link",)
    rewards["body_ang_vel"].weight = -0.05  # as G1
    rewards["pose"].params["asset_cfg"] = motor_joints()
    rewards["pose"].params["std_standing"] = {".*": 0.05}
    rewards["pose"].params["std_walking"] = POSE_STD.copy()
    rewards["pose"].params["std_running"] = POSE_STD_RUNNING.copy()
    # Reward a timed swing only while the other foot supports the robot.
    rewards["air_time"].func = single_support_air_time
    rewards["air_time"].weight = 0.5
    rewards["air_time"].params["command_threshold"] = 0.1

    for name in ("foot_clearance", "foot_slip"):
        rewards[name].params["asset_cfg"].site_names = FOOT_SITES
    for name in ("foot_clearance", "foot_swing_height"):
        rewards[name].params["target_height"] = FOOT_CLEARANCE
    del rewards["angular_momentum"]  # needs a sensor the robot model does not have
    # G1's self-collision cost is omitted: this model disables self-collision.

    if navigation:
        # Score XY and yaw directly. The existing upright and roll/pitch terms
        # continue to penalize tipping while leaving commanded yaw free.
        rewards["track_linear_velocity"].func = track_planar_velocity
        rewards["track_linear_velocity"].weight = 3.0
        rewards["track_linear_velocity"].params["std"] = 0.25
        rewards["track_angular_velocity"].func = track_yaw_rate
        rewards["track_angular_velocity"].weight = 3.0
        rewards["track_angular_velocity"].params["std"] = 0.30

    # Resets: fitted crouch, zero velocity, including consistent passive joints.
    cfg.events["reset_base"].params["pose_range"] = {}

    # First debugging run: no pushes and no randomization.
    for name in ("push_robot", "foot_friction", "encoder_bias", "base_com"):
        del cfg.events[name]

    # The floating root and torso origins coincide. On this flat ground, root Z
    # therefore measures torso height; stop collapsed poses even if still upright.
    del cfg.terminations["out_of_terrain_bounds"]
    cfg.terminations["torso_too_low"] = TerminationTermCfg(
        func=envs_mdp.root_height_below_minimum,
        params={"minimum_height": MIN_TORSO_HEIGHT},
    )
    cfg.curriculum = {}

    cfg.viewer.body_name = "torso_link"

    if play:
        cfg.scene.num_envs = 1
        cfg.episode_length_s = int(1e9)  # no time limit while watching

    return cfg
