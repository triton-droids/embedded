"""Walking reset fitted to the current MJCF; validate after geometry changes.

Knees bend 0.25 rad, elbows 0.08 rad. Hip/ankle angles and root height
were solved with native MuJoCo kinematics: level soles 1 mm above ground,
closed connect constraints, and COM centered fore/aft between the soles.
The zero-pose robot adapter remains available for its original standing check.
"""

import re
import xml.etree.ElementTree as ET
from copy import deepcopy

from mjlab.entity import EntityCfg

from humanoid_mjlab.robot import ROBOT_CFG, ROBOT_XML

JOINT_TARGET_MARGIN = 0.03  # radians inside mechanical stops
# Conservative crank travel, within the linkage's driven branch near zero.
ANKLE_CRANK_TARGET_RANGE = (-0.5, 0.5)
CROUCH_HEIGHT = 0.6614941694814562
CROUCH_JOINT_POS = {
    "left_hip1_joint": 0.055681659758865375,
    "left_hip2_joint": 0.0006222430907445142,
    "left_thigh_joint": 0.0,
    "left_knee_joint": -0.25,
    "left_ankle_joint": 0.19430855446784598,
    "left_ankle_crank_joint": 0.191695084229945,
    "left_ankle_rod_joint": -0.19158933905343925,
    "right_hip1_joint": 0.15196484128440685,
    "right_hip2_joint": -0.0019925551217389033,
    "right_thigh_joint": 0.0,
    "right_knee_joint": -0.25,
    "right_ankle_joint": 0.09788449251524589,
    "right_ankle_crank_joint": 0.09657612538805598,
    "right_ankle_rod_joint": -0.09652195805641024,
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.08,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.08,
}


def walking_robot_cfg() -> EntityCfg:
    cfg = deepcopy(ROBOT_CFG)
    cfg.init_state = EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, CROUCH_HEIGHT),
        joint_pos=CROUCH_JOINT_POS.copy(),
        joint_vel={".*": 0.0},
    )
    # A crouch cannot rely on the knee extension stops for support. These gains
    # hold it in the native 10 s check without changing peak torque limits.
    for actuator in cfg.articulation.actuators:
        if any(
            "hip" in n or "knee" in n or "thigh" in n
            for n in actuator.target_names_expr
        ):
            actuator.stiffness = 400.0
            actuator.damping = 10.0
        elif any("ankle" in n for n in actuator.target_names_expr):
            actuator.stiffness = 300.0
            actuator.damping = 6.0
    return cfg


def joint_target_bounds(patterns: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    """Absolute position bounds, applied AFTER action scaling and default offset.

    Read limits from the MJCF rather than duplicating them. Continuous shoulder
    pitch joints have no mechanical stop; passive linkage joints get no actions.
    """
    bounds = {}
    for joint in ET.parse(ROBOT_XML).getroot().findall(".//worldbody//joint"):
        name = joint.attrib["name"]
        if not any(re.fullmatch(pattern, name) for pattern in patterns):
            continue
        if name.endswith("_ankle_crank_joint"):
            bounds[name] = ANKLE_CRANK_TARGET_RANGE
        elif "range" in joint.attrib and joint.get("limited") != "false":
            lo, hi = map(float, joint.attrib["range"].split())
            bounds[name] = (lo + JOINT_TARGET_MARGIN, hi - JOINT_TARGET_MARGIN)
        if name in bounds:
            lo, hi = bounds[name]
            if not lo < CROUCH_JOINT_POS[name] < hi:
                raise ValueError(f"Crouch target outside margins: {name}")
    return bounds
