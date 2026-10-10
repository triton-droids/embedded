"""Shared robot hardware configuration.

Keep robot-wide motor facts here so tools do not drift apart on IDs, models,
joint names, direction signs, and limits.
"""

from __future__ import annotations

from dataclasses import dataclass


CAN_CHANNEL = "can0"
BITRATE = 1_000_000
DEFAULT_MOTOR_MODEL = "rs-03"

REAL_JOINT_ORDER = [
    "left_hip1_joint",
    "left_hip2_joint",
    "left_thigh_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip1_joint",
    "right_hip2_joint",
    "right_thigh_joint",
    "right_knee_joint",
    "right_ankle_joint",
]

POLICY_JOINT_ORDER = [
    "left_hip1_joint",
    "right_hip1_joint",
    "left_hip2_joint",
    "right_hip2_joint",
    "left_thigh_joint",
    "right_thigh_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_ankle_joint",
    "right_ankle_joint",
]

JOINT_TO_MOTOR_ID = {
    "left_hip1_joint": 1,
    "left_hip2_joint": 2,
    "left_thigh_joint": 3,
    "left_knee_joint": 4,
    "left_ankle_joint": 5,
    "right_hip1_joint": 6,
    "right_hip2_joint": 7,
    "right_thigh_joint": 8,
    "right_knee_joint": 9,
    "right_ankle_joint": 10,
}

MOTOR_MODEL_BY_ID = {
    1: "rs-04",
    2: "rs-03",
    3: "rs-03",
    4: "rs-04",
    5: "rs-02",
    6: "rs-04",
    7: "rs-03",
    8: "rs-03",
    9: "rs-04",
    10: "rs-02",
}

# Indexed by motor ID 1..10.
INVERSION_ARRAY = [-1, 1, 1, 1, 1, 1, -1, -1, -1, -1]
INVERSION_BY_ID = {mid: INVERSION_ARRAY[mid - 1] for mid in range(1, len(INVERSION_ARRAY) + 1)}

JOINT_LIMITS_RAD_BY_JOINT = {
    "left_hip1_joint": (-1.57, 1.57),
    "left_hip2_joint": (-1.57, 0.436332),
    "left_thigh_joint": (-0.785398, 0.785398),
    "left_knee_joint": (-2.0944, 0.0),
    "left_ankle_joint": (-0.6, 0.6),
    "right_hip1_joint": (-1.57, 1.57),
    "right_hip2_joint": (-0.436332, 1.57),
    "right_thigh_joint": (-0.785398, 0.785398),
    "right_knee_joint": (-2.0944, 0.0),
    "right_ankle_joint": (-0.6, 0.6),
}

JOINT_NAME_BY_ID = {mid: joint for joint, mid in JOINT_TO_MOTOR_ID.items()}
JOINT_LIMITS_BY_ID = {
    mid: JOINT_LIMITS_RAD_BY_JOINT[joint_name]
    for joint_name, mid in JOINT_TO_MOTOR_ID.items()
}

DEFAULT_JOINT_POS_REAL_RAD_BY_JOINT = {
    "left_hip1_joint": 0.4,
    "left_hip2_joint": 0.0,
    "left_thigh_joint": 0.0,
    "left_knee_joint": -0.8,
    "left_ankle_joint": 0.4,
    "right_hip1_joint": 0.4,
    "right_hip2_joint": 0.0,
    "right_thigh_joint": 0.0,
    "right_knee_joint": -0.8,
    "right_ankle_joint": 0.4,
}

POLICY_KP_BY_JOINT = {
    "left_hip1_joint": 300.0,
    "left_hip2_joint": 200.0,
    "left_thigh_joint": 100.0,
    "left_knee_joint": 100.0,
    "left_ankle_joint": 120.0,
    "right_hip1_joint": 200.0,
    "right_hip2_joint": 300.0,
    "right_thigh_joint": 100.0,
    "right_knee_joint": 100.0,
    "right_ankle_joint": 120.0,
}

POLICY_KD_BY_JOINT = {
    "left_hip1_joint": 20.0,
    "left_hip2_joint": 5.0,
    "left_thigh_joint": 2.0,
    "left_knee_joint": 20.0,
    "left_ankle_joint": 0.8,
    "right_hip1_joint": 5.0,
    "right_hip2_joint": 15.0,
    "right_thigh_joint": 2.0,
    "right_knee_joint": 20.0,
    "right_ankle_joint": 1.0,
}

# Conservative defaults for manual health checks, intentionally lower than
# policy-control gains.
HEALTH_CHECK_KP_BY_ID = {
    1: 30.0,
    2: 30.0,
    3: 20.0,
    4: 30.0,
    5: 30.0,
    6: 30.0,
    7: 30.0,
    8: 20.0,
    9: 30.0,
    10: 30.0,
}
HEALTH_CHECK_KD_BY_ID = {mid: 0.5 for mid in range(1, 11)}

# Per-motor MIT gains used by dataset collection. These are intentionally kept
# separate from policy and manual health-check gains.
DATASET_GAINS_BY_ID = {
    1: (250.0, 5.0),
    2: (250.0, 5.0),
    3: (100.0, 2.0),
    4: (150.0, 5.0),
    5: (120.0, 0.8),
    6: (250.0, 5.0),
    7: (250.0, 5.0),
    8: (100.0, 2.0),
    9: (150.0, 5.0),
    10: (120.0, 1.0),
}

DEFAULT_JOINT_POS_BY_ID = {
    mid: DEFAULT_JOINT_POS_REAL_RAD_BY_JOINT[joint_name]
    for joint_name, mid in JOINT_TO_MOTOR_ID.items()
}


@dataclass(frozen=True)
class MotorHardware:
    motor_id: int
    joint_name: str
    model: str
    direction: int
    limit_lo: float
    limit_hi: float


def motor_hardware(motor_id: int) -> MotorHardware:
    """Return the shared hardware facts for a motor ID."""
    joint_name = JOINT_NAME_BY_ID.get(motor_id, f"motor_{motor_id}")
    limit_lo, limit_hi = JOINT_LIMITS_BY_ID.get(motor_id, (-float("inf"), float("inf")))
    direction = 1 if INVERSION_BY_ID.get(motor_id, 1) >= 0 else -1
    return MotorHardware(
        motor_id=motor_id,
        joint_name=joint_name,
        model=MOTOR_MODEL_BY_ID.get(motor_id, DEFAULT_MOTOR_MODEL),
        direction=direction,
        limit_lo=limit_lo,
        limit_hi=limit_hi,
    )


def apply_shared_hardware_config(cfg: dict) -> dict:
    """Overlay shared hardware facts onto a run configuration dict."""
    cfg.update(
        {
            "can_channel": CAN_CHANNEL,
            "bitrate": BITRATE,
            "real_joint_order": list(REAL_JOINT_ORDER),
            "policy_joint_order": list(POLICY_JOINT_ORDER),
            "joint_to_motor_id": dict(JOINT_TO_MOTOR_ID),
            "motor_model_by_id": dict(MOTOR_MODEL_BY_ID),
            "default_motor_model": DEFAULT_MOTOR_MODEL,
            "inversion_array": list(INVERSION_ARRAY),
            "joint_limits_rad_by_joint": {
                joint: list(limits)
                for joint, limits in JOINT_LIMITS_RAD_BY_JOINT.items()
            },
            "default_joint_pos_real_rad_by_joint": dict(DEFAULT_JOINT_POS_REAL_RAD_BY_JOINT),
            "kp_by_joint": dict(POLICY_KP_BY_JOINT),
            "kd_by_joint": dict(POLICY_KD_BY_JOINT),
        }
    )
    return cfg
