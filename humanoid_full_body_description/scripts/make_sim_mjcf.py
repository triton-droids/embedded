#!/usr/bin/env python3
"""Generate mjcf/full_body_sim.xml, the full-body robot for humanoid_sim, from the simplified URDF.

humanoid_sim loads a robot MJCF and adds floor, motors, IMU sensors, head and gantry around it
(simulation/humanoid_sim/humanoid_sim/model_builder.py). It expects what the tracking policy's
MJCF (motor_control_hybrid/models/*/chrobot_16kg_candidate.xml) provides:

- a free-floating body `floating_base` with an `imu` site identity-aligned to it,
- a body `torso` with a `top` site (the head is mounted there),
- leg joints named as in the motor registry (left_hip1_joint ... right_ankle_joint),
- sole-centre sites `left_foot` / `right_foot` and box contacts `*_foot_collision_box`.

This script builds exactly that from urdf/full_body.urdf.xacro:
- `floating_base` sits at the torso_link origin (centre of the torso's bottom face, +z up),
- the leg joints get the policy MJCF's damping (1.0) and armature (0.01); their names, axis signs and
  ranges already match it in the URDF,
- the arms, which have no motors in the registry, get position servos that hold ctrl (0 = hanging down),
- each sole gets a box contact sized from the foot's mesh (the training MJCF used boxes too); the
  foot hull no longer touches the floor,
- self-contact pairs that the SRDF disables (adjacent or never colliding) are excluded, as in
  humanoid_full_body_moveit_config/scripts/mujoco_sim.py.

It also writes the Pinocchio model for the model-based walking controller (walking_controller,
`controller:=model_walk`): urdf/full_body_wbc.urdf with the arm joints fixed (arms ride on the torso as rigid
mass), frames renamed to the controller's `torso` / `left_foot` / `right_foot`, and
urdf/full_body_wbc.dynamics.json (MuJoCo base to Pinocchio root, leg armature/damping, sole corners).
With the arms driven too (`robot:=full_body_arms`): urdf/full_body_wbc_arms.* (all 20 joints) and
config/motors_full_body.yaml (leg registry plus placeholder arm motors; humanoid_sim then replaces the arm
position servos with the same MIT-law motors as the legs).

Run after changing the URDF (needs ROS xacro and the mujoco Python module, e.g. from rosenv):
  rosenv/bin/python humanoid_full_body_description/scripts/make_sim_mjcf.py
"""

import argparse
import json
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

PACKAGE = Path(__file__).resolve().parent.parent
REPO = PACKAGE.parent
SRDF = REPO / "humanoid_full_body_moveit_config/config/full_body.srdf"
LEG_JOINTS = [f"{s}_{j}_joint" for s in ("left", "right") for j in ("hip1", "hip2", "thigh", "knee", "ankle")]
RENAME_BODY = {"torso_link": "torso", "pelvis_link": "hip", "left_foot_link": "left_foot",
               "right_foot_link": "right_foot"}
ARM_JOINTS = [f"{s}_{j}_joint" for s in ("left", "right")
              for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "upper_arm_twist", "elbow")]
ARM_DAMPING, ARM_ARMATURE = 0.5, 0.01
ARM_MOTOR_MODEL = "rs-02"  # placeholder
LEG_REGISTRY = REPO / "humanoid_control/motor_control_hybrid/config/motors.yaml"
ARM_SERVO_KP = 40.0   # N*m/rad, placeholder: only holds the arms at their commanded pose
ARM_SERVO_KV = 2.0    # N*m*s/rad
ARM_FORCE_LIMIT = 20.0
SOLE_SLICE = 0.01     # m: vertices this close to the lowest point form the sole
TOP_HEIGHT = 0.385    # m, top of the torso frame above torso_link


def expand_xacro(xacro_file):
    return subprocess.run(["xacro", str(xacro_file)], check=True, capture_output=True, text=True).stdout


def load_urdf_spec(xacro_file):
    urdf = expand_xacro(xacro_file)
    robot = ET.fromstring(urdf)
    for mesh in robot.iter("mesh"):  # relative to meshdir: visual/<link>.stl, collision/<link>.stl
        mesh.set("filename", mesh.get("filename").split("/meshes/", 1)[1])
    compiler = ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler")
    compiler.attrib.update(meshdir=str(PACKAGE / "meshes"), strippath="false", fusestatic="false",
                           discardvisual="false", balanceinertia="true")
    spec = mujoco.MjSpec.from_string(ET.tostring(robot, encoding="unicode"))
    # The URDF importer names mesh assets by file basename, so visual/<link>.stl and collision/<link>.stl
    # collapse into one asset (the visual mesh). Give every collision geom its own asset.
    for geom in spec.geoms:
        if geom.contype:  # the importer marks collision geoms contype=1, visual ones 0
            link = geom.parent.name
            if spec.mesh(f"{link}_collision") is None:
                spec.add_mesh(name=f"{link}_collision", file=f"collision/{link}.stl")
            geom.meshname = f"{link}_collision"
    return spec


WBC_RENAME = {"torso_link": "torso", "left_foot_link": "left_foot", "right_foot_link": "right_foot"}


def write_wbc_model(urdf_text, model, data, out_dir, arms=False):
    """Pinocchio URDF (controller frame names) and its .dynamics.json.

    arms=False: full_body_wbc.* with the arm joints fixed (10 leg joints).
    arms=True: full_body_wbc_arms.* with all 20 joints; continuous arm joints become revolute
    (Pinocchio models a continuous joint with nq = 2, the controller expects nq = nv)."""
    robot = ET.fromstring(urdf_text)
    for link in robot.findall("link"):
        link.set("name", WBC_RENAME.get(link.get("name"), link.get("name")))
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    for joint in robot.findall("joint"):
        for tag in ("parent", "child"):
            element = joint.find(tag)
            element.set("link", WBC_RENAME.get(element.get("link"), element.get("link")))
        if joint.get("type") == "continuous" and arms:
            joint.set("type", "revolute")
            ET.SubElement(joint, "limit", lower=str(-np.pi), upper=str(np.pi), effort="0", velocity="0")
        elif joint.get("type") != "fixed" and joint.get("name") not in LEG_JOINTS and not arms:
            joint.set("type", "fixed")
            for tag in ("axis", "limit"):
                if joint.find(tag) is not None:
                    joint.remove(joint.find(tag))
    corners = {}
    for side in ("left", "right"):
        geom = model.geom(f"{side}_foot_collision_box").id
        body = model.body(f"{side}_foot").id
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, model.geom_quat[geom])
        sx, sy, sz = model.geom_size[geom]
        local = [model.geom_pos[geom] + rot.reshape(3, 3) @ np.array([x * sx, y * sy, -sz])
                 for x in (1, -1) for y in (1, -1)]
        assert model.geom_bodyid[geom] == body
        corners[f"{side}_foot"] = np.array(local).T.tolist()  # 3 x 4, foot frame
    dynamics = {
        "source": "humanoid_full_body_description/mjcf/full_body_sim.xml",
        # floating_base sits at the torso_link origin with identity rotation.
        "mujoco_base_to_pin_root": {"translation": [0.0, 0.0, 0.0], "rotation": np.eye(3).tolist()},
        "joints": {**{name: {"armature": 0.01, "damping": 1.0} for name in LEG_JOINTS},
                   **({name: {"armature": ARM_ARMATURE, "damping": ARM_DAMPING} for name in ARM_JOINTS}
                      if arms else {})},
        "contact_points": corners,
    }
    stem = "full_body_wbc_arms" if arms else "full_body_wbc"
    note = "All 20 joints." if arms else "Arm joints fixed."
    (out_dir / f"{stem}.urdf").write_text(
        "<?xml version=\"1.0\"?>\n<!-- Generated by scripts/make_sim_mjcf.py: Pinocchio model for walking_controller. "
        f"{note} Do not edit. -->\n" + ET.tostring(robot, encoding="unicode"))
    (out_dir / f"{stem}.dynamics.json").write_text(json.dumps(dynamics, indent=2) + "\n")


def write_arm_registry(urdf_text, out_file):
    """Motor registry with the 10 leg motors of motor_control_hybrid/config/motors.yaml plus 10 arm motors.

    For simulation only (robot:=full_body_arms): the arm entries are placeholders (motor model, CAN ids,
    direction and gains are not the real arm hardware's)."""
    import yaml
    legs = yaml.safe_load(LEG_REGISTRY.read_text())
    params = legs["motor_control_node"]["ros__parameters"]
    limits = {}
    for joint in ET.fromstring(urdf_text).findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") in ARM_JOINTS:
            limits[joint.get("name")] = ((float(limit.get("lower")), float(limit.get("upper")))
                                         if limit is not None else (-np.pi, np.pi))
    for i, name in enumerate(ARM_JOINTS):
        params["motors"][name] = {
            "can_interface": "can2", "master_id": 255, "motor_id": 11 + i, "model": ARM_MOTOR_MODEL,
            "actuator_type": None, "direction": 1, "encoder_offset_rad": 0.0,
            "max_torque_nm": ARM_FORCE_LIMIT, "min_position": round(limits[name][0], 6),
            "max_position": round(limits[name][1], 6), "kp": ARM_SERVO_KP, "kd": ARM_SERVO_KV}
    header = ("# Generated by humanoid_full_body_description/scripts/make_sim_mjcf.py: the leg motors of\n"
              "# motor_control_hybrid/config/motors.yaml plus 10 arm motors. SIMULATION ONLY: the arm entries\n"
              "# (model, CAN ids, direction, gains) are placeholders, not the real arm hardware. Do not edit.\n")
    out_file.parent.mkdir(parents=True, exist_ok=True)
    out_file.write_text(header + yaml.safe_dump(legs, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=PACKAGE / "mjcf/full_body_sim.xml")
    args = parser.parse_args()

    robot = load_urdf_spec(PACKAGE / "urdf/full_body.urdf.xacro")
    spec = mujoco.MjSpec()
    spec.modelname = "full_body_sim"
    spec.compiler.autolimits = True
    spec.meshdir = "../meshes"  # humanoid_sim overrides this with its mesh_dir parameter
    spec.option.timestep = 0.005  # as the policy MJCF
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC

    base = spec.worldbody.add_body(name="floating_base", pos=[0, 0, 0.7])
    base.add_freejoint()
    base.add_site(name="imu")
    base.add_frame().attach_body(robot.body("torso_link"), "", "")
    for old, new in RENAME_BODY.items():
        spec.body(old).name = new
    spec.body("torso").add_site(name="top", pos=[0, 0, TOP_HEIGHT], size=[0.01, 0, 0], rgba=[0, 1, 0, 1])

    # Visual meshes never collide; collision hulls collide with each other (minus exclusions) and the floor.
    for geom in spec.geoms:
        visual = geom.meshname and spec.mesh(geom.meshname).file.startswith("visual/")
        geom.group, geom.contype, geom.conaffinity = (2, 0, 0) if visual else (3, 1, 1)

    for joint in spec.joints:
        if joint.name in LEG_JOINTS:
            joint.damping, joint.armature = 1.0, 0.01
        elif joint.type == mujoco.mjtJoint.mjJNT_HINGE:  # arms
            joint.damping, joint.armature = ARM_DAMPING, ARM_ARMATURE
            servo = spec.add_actuator(name=f"{joint.name}_servo", target=joint.name,
                                      trntype=mujoco.mjtTrn.mjTRN_JOINT)
            servo.set_to_position(kp=ARM_SERVO_KP, kv=ARM_SERVO_KV)
            servo.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            servo.forcerange = [-ARM_FORCE_LIMIT, ARM_FORCE_LIMIT]

    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    for side in ("left", "right"):
        foot = spec.body(f"{side}_foot")
        # Sole box from the foot's collision mesh: the sole's footprint, 15 mm thick at the bottom.
        geom_id = next(g for g in range(model.ngeom)
                       if model.geom_bodyid[g] == model.body(f"{side}_foot").id and model.geom_contype[g])
        mesh = model.geom_dataid[geom_id]
        verts = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        # Box axes follow the world at the zero pose (soles flat, toes along +y), not the foot's frame:
        # the leg zero offsets leave the left foot frame rotated about 30 deg.
        world = verts @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
        # Only the sole: the foot link also carries the heel linkage rods, which reach back and up.
        world = world[world[:, 2] < world[:, 2].min() + SOLE_SLICE]
        lo, hi = world.min(0), world.max(0)
        half = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.0075]
        body_id = model.body(f"{side}_foot").id
        body_rot, body_pos = data.xmat[body_id].reshape(3, 3), data.xpos[body_id]
        to_body = lambda point: (np.asarray(point) - body_pos) @ body_rot
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, body_rot.T.flatten())
        centre = to_body([(hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, lo[2] + 0.0075])
        sole = to_body([(hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, lo[2]])
        foot.add_geom(name=f"{side}_foot_collision_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=half,
                      pos=centre, quat=quat, group=3, contype=1, conaffinity=1)
        foot.add_site(name=f"{side}_foot", pos=sole, quat=quat, size=[0.01, 0, 0])
        for geom in spec.geoms:  # the hull stays for self-collision only
            if geom.parent.name == f"{side}_foot" and geom.meshname and not geom.name:
                if spec.mesh(geom.meshname).file.startswith("collision/"):
                    geom.contype, geom.conaffinity = 2, 2

    links = {b.name for b in spec.bodies}
    rename = {**RENAME_BODY}
    for pair in ET.parse(SRDF).getroot().findall("disable_collisions"):
        a, b = (rename.get(pair.get(k), pair.get(k)) for k in ("link1", "link2"))
        if a in links and b in links:
            exclude = spec.add_exclude()
            exclude.bodyname1, exclude.bodyname2 = a, b

    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    xml = spec.to_xml()
    xml = xml.replace(f'meshdir="{PACKAGE / "meshes"}"', 'meshdir="../meshes"')
    # attach_body leaves an empty nested default that MjSpec.from_file rejects ("empty class name").
    xml = re.sub(r"\n  <default>\s*<default/>\s*</default>\n", "\n", xml)
    header = ("<!-- Generated by humanoid_full_body_description/scripts/make_sim_mjcf.py from "
              "urdf/full_body.urdf.xacro. Do not edit. -->\n")
    args.output.write_text(header + xml)
    urdf = expand_xacro(PACKAGE / "urdf/full_body.urdf.xacro")
    write_wbc_model(urdf, model, data, PACKAGE / "urdf")
    write_wbc_model(urdf, model, data, PACKAGE / "urdf", arms=True)
    write_arm_registry(urdf, PACKAGE / "config/motors_full_body.yaml")
    mass = sum(model.body_mass)
    print(f"{args.output}: {model.nbody} bodies, {model.njnt} joints ({len(LEG_JOINTS)} legs + "
          f"{model.nu} arm servos), {mass:.2f} kg, {data.ncon} contacts at zero")


if __name__ == "__main__":
    main()
