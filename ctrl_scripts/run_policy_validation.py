#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Validation utilities for run_policy configuration.
"""

from __future__ import annotations

from typing import Any


EXPECTED_JOINT_COUNT = 10


def require_key(cfg: dict[str, Any], key: str, ctx: str = "config") -> Any:
    if key not in cfg:
        raise KeyError(f"Missing required key '{ctx}.{key}' in config.")
    return cfg[key]


def validate_config(cfg: dict[str, Any]) -> None:
    required_top = [
        "can_channel",
        "bitrate",
        "control_hz",
        "default_policy_path",
        "torch_device",
        "real_joint_order",
        "policy_joint_order",
        "joint_to_motor_id",
        "motor_model_by_id",
        "default_motor_model",
        "inversion_array",
        "joint_limits_rad_by_joint",
        "default_joint_pos_real_rad_by_joint",
        "kp",
        "kd",
        "kp_by_joint",
        "kd_by_joint",
        "max_vel_rad_s",
        "max_vel_rad_s_by_joint",
        "action_scale",
        "action_scale_by_joint",
        "policy_action_clip",
        "use_soft_joint_limits",
        "soft_joint_limit_factor",
        "dof_vel_scale",
        "ang_vel_scale",
        "command_obs",
        "use_phase_obs",
        "gait_period_s",
        "freeze_phase_when_standing",
        "stand_phase_lin_threshold",
        "stand_phase_yaw_threshold",
        "stand_phase_value",
        "frame_stack",
        "stack_frame_major",
        "ankle_joint_names",
        "ankle_mapping",
        "imu",
        "status_print_hz",
    ]
    missing = [k for k in required_top if k not in cfg]
    if missing:
        raise ValueError(f"Missing required config keys: {', '.join(missing)}")

    for nested, nested_required in (
        (
            "ankle_mapping",
            ["link_lengths", "theta2_offset_deg", "t4_to_motor_offset_deg", "ankle_to_theta2_sign"],
        ),
        (
            "imu",
            ["enabled", "source", "port", "baud", "rate_hz", "include_all", "wait_for_first_sample_s", "gyro_sign", "accel_sign"],
        ),
    ):
        if not isinstance(cfg[nested], dict):
            raise ValueError(f"Config key '{nested}' must be an object.")
        missing_nested = [k for k in nested_required if k not in cfg[nested]]
        if missing_nested:
            raise ValueError(f"Missing required config keys under '{nested}': {', '.join(missing_nested)}")

    real_order = cfg["real_joint_order"]
    policy_order = cfg["policy_joint_order"]
    if len(real_order) != EXPECTED_JOINT_COUNT or len(set(real_order)) != EXPECTED_JOINT_COUNT:
        raise ValueError(f"real_joint_order must contain exactly {EXPECTED_JOINT_COUNT} unique joints.")
    if len(policy_order) != EXPECTED_JOINT_COUNT or len(set(policy_order)) != EXPECTED_JOINT_COUNT:
        raise ValueError(f"policy_joint_order must contain exactly {EXPECTED_JOINT_COUNT} unique joints.")
    if set(real_order) != set(policy_order):
        raise ValueError("real_joint_order and policy_joint_order must contain the same joints.")

    for joint_name in real_order:
        if joint_name not in cfg["joint_to_motor_id"]:
            raise ValueError(f"joint_to_motor_id is missing joint '{joint_name}'.")
        if joint_name not in cfg["joint_limits_rad_by_joint"]:
            raise ValueError(f"joint_limits_rad_by_joint is missing joint '{joint_name}'.")
        if joint_name not in cfg["default_joint_pos_real_rad_by_joint"]:
            raise ValueError(f"default_joint_pos_real_rad_by_joint is missing joint '{joint_name}'.")

    if len(cfg["command_obs"]) != 3:
        raise ValueError("command_obs must be length 3: [vx, vy, yaw_rate].")

