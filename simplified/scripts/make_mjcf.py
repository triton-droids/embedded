#!/usr/bin/env python3
"""Generate mjcf/full_body.xml (robot with torque motors) from urdf/full_body.urdf.

Adapted from humanoid_full_body_description/scripts/make_sim_mjcf.py on sim-experimental-cyclonedds, with the
motors written into the MJCF instead of added by humanoid_sim at runtime:

- `floating_base` (free joint) at the torso_link origin, standing on the floor at the zero pose,
- one <motor> per motor (gear 1: ctrl is torque in N*m, ctrlrange = the motor's peak torque):
  hip pitch / knee RS-04 120, hip roll / hip yaw RS-03 60, shoulder pitch / roll RS-03 60,
  shoulder yaw / elbow RS-02 17,
- ankle: the RS-02 drives the four-bar crank (`*_ankle_crank_joint`, the motor axis), not the foot. The crank and rod
  are passive hinges and an equality connect pins the rod's lower eye to the heel lever, so MuJoCo transmits the
  17 N*m motor torque to the foot through the exact linkage geometry (the foot torque varies with the ankle angle),
- the motorless `*_upper_arm_twist_joint` is welded,
- each sole gets a 15 mm box contact from the foot mesh; the robot collides with the floor only (no self-collision).

Run after changing the URDF:  python3 scripts/make_mjcf.py
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

PACKAGE = Path(__file__).resolve().parent.parent
URDF = PACKAGE / "urdf/full_body.urdf"
OUTPUT = PACKAGE / "mjcf/full_body.xml"

PEAK_TORQUE_NM = {"rs-02": 17.0, "rs-03": 60.0, "rs-04": 120.0}  # peak, not continuous
# actuated joint -> motor model, in registry order (config/motors_full_body.yaml)
MOTORS = {}
for s in ("left", "right"):
    MOTORS.update({f"{s}_hip1_joint": "rs-04", f"{s}_hip2_joint": "rs-03", f"{s}_thigh_joint": "rs-03",
                   f"{s}_knee_joint": "rs-04", f"{s}_ankle_crank_joint": "rs-02"})
for s in ("left", "right"):
    MOTORS.update({f"{s}_shoulder_pitch_joint": "rs-03", f"{s}_shoulder_roll_joint": "rs-03",
                   f"{s}_shoulder_yaw_joint": "rs-02", f"{s}_elbow_joint": "rs-02"})
LEG_JOINTS = [f"{s}_{j}_joint" for s in ("left", "right") for j in ("hip1", "hip2", "thigh", "knee", "ankle")]
LINKAGE_JOINTS = [f"{s}_ankle_{j}_joint" for s in ("left", "right") for j in ("crank", "rod")]
WELDED_JOINTS = [f"{s}_upper_arm_twist_joint" for s in ("left", "right")]
LEG_DAMPING, LEG_ARMATURE = 1.0, 0.01       # policy MJCF
ARM_DAMPING, ARM_ARMATURE = 0.5, 0.01
LINKAGE_DAMPING, LINKAGE_ARMATURE = 0.01, 0.001
ANKLE_MOTOR_ARMATURE = 0.01                  # rotor inertia sits on the crank (motor) joint
LINKAGE_SOLREF = 0.01                        # s, twice the timestep
# Near-hard loop: with MuJoCo's default solimp (0.9, 0.95) the connect stretches about 6 mm under body weight, the
# foot turns twice as far as the crank and the robot tips over; with this solimp the stretch stays under 0.3 mm.
LINKAGE_SOLIMP = [0.99, 0.999, 0.001, 0.5, 2.0]
SOLE_SLICE = 0.01                            # m, vertices this close to the lowest point form the sole
SOLE_HALF_THICKNESS = 0.0075
FLOOR_GROUP_BIT, ROBOT_GROUP_BIT = 1, 2      # robot geoms touch the floor, not each other


def load_urdf_spec():
    robot = ET.parse(URDF).getroot()
    for mesh in robot.iter("mesh"):  # relative to meshdir: visual/<link>.stl, collision/<link>.stl
        mesh.set("filename", mesh.get("filename").split("/meshes/", 1)[1])
    compiler = ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler")
    compiler.attrib.update(meshdir=str(PACKAGE / "meshes"), strippath="false", fusestatic="false",
                           discardvisual="false", balanceinertia="true")
    spec = mujoco.MjSpec.from_string(ET.tostring(robot, encoding="unicode"))
    # The importer names mesh assets by file basename, so visual/ and collision/<link>.stl collapse into one.
    for geom in spec.geoms:
        if geom.contype:  # importer: collision geoms contype=1, visual 0
            link = geom.parent.name
            if spec.mesh(f"{link}_collision") is None:
                spec.add_mesh(name=f"{link}_collision", file=f"collision/{link}.stl")
            geom.meshname = f"{link}_collision"
    return spec


def main():
    robot = load_urdf_spec()
    spec = mujoco.MjSpec()
    spec.modelname = "full_body"
    spec.compiler.autolimits = True
    spec.meshdir = "../meshes"
    spec.option.timestep = 0.005
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC

    base = spec.worldbody.add_body(name="floating_base")
    base.add_freejoint(name="floating_base_joint")
    base.add_site(name="imu")
    base.add_frame().attach_body(robot.body("torso_link"), "", "")

    for geom in spec.geoms:
        visual = geom.meshname and spec.mesh(geom.meshname).file.startswith("visual/")
        geom.group, geom.contype, geom.conaffinity = (2, 0, 0) if visual else (3, ROBOT_GROUP_BIT, FLOOR_GROUP_BIT)

    for joint in [j for j in spec.joints if j.name in WELDED_JOINTS]:
        spec.delete(joint)
    for joint in spec.joints:
        if joint.name in LINKAGE_JOINTS:
            joint.damping, joint.armature = LINKAGE_DAMPING, LINKAGE_ARMATURE
            joint.limited = mujoco.mjtLimited.mjLIMITED_FALSE
        elif joint.name in LEG_JOINTS:
            joint.damping, joint.armature = LEG_DAMPING, LEG_ARMATURE
        elif joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            joint.damping, joint.armature = ARM_DAMPING, ARM_ARMATURE
    for side in ("left", "right"):
        # The ankle motor turns the crank; the foot joint itself is passive (its damping stays as the bearing's).
        spec.joint(f"{side}_ankle_crank_joint").armature = ANKLE_MOTOR_ARMATURE
        spec.joint(f"{side}_ankle_joint").armature = 0.0

    for eq in list(spec.equalities):  # mimic equalities from the importer: replaced by the closed loop
        spec.delete(eq)
    for side in ("left", "right"):
        end = spec.body(f"{side}_ankle_rod_end")
        anchor = list(end.pos)
        spec.delete(end)
        spec.add_equality(type=mujoco.mjtEq.mjEQ_CONNECT, name=f"{side}_ankle_linkage",
                          name1=f"{side}_ankle_rod_link", name2=f"{side}_foot_link",
                          objtype=mujoco.mjtObj.mjOBJ_BODY, data=anchor + [0.0] * 8,
                          solref=[LINKAGE_SOLREF, 1.0], solimp=LINKAGE_SOLIMP)
        for link in (f"{side}_ankle_crank_link", f"{side}_ankle_rod_link"):
            for geom in spec.body(link).geoms:
                geom.contype, geom.conaffinity = 0, 0

    for joint, model_name in MOTORS.items():
        peak = PEAK_TORQUE_NM[model_name]
        name = joint.replace("_crank_joint", "_joint").replace("_joint", "_motor")  # left_ankle_motor turns the crank
        spec.add_actuator(name=name, target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gear=[1, 0, 0, 0, 0, 0], ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                          ctrlrange=[-peak, peak])

    # Sole boxes; the foot hull (which also carries the heel lever) no longer touches the floor.
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    sole_z = []
    for side in ("left", "right"):
        foot = spec.body(f"{side}_foot_link")
        body_id = model.body(f"{side}_foot_link").id
        geom_id = next(g for g in range(model.ngeom) if model.geom_bodyid[g] == body_id and model.geom_contype[g])
        mesh = model.geom_dataid[geom_id]
        verts = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        with np.errstate(all="ignore"):  # spurious float warnings from numpy's Accelerate matmul on macOS
            world = verts.astype(float) @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
        world = world[world[:, 2] < world[:, 2].min() + SOLE_SLICE]
        lo, hi = world.min(0), world.max(0)
        body_rot, body_pos = data.xmat[body_id].reshape(3, 3), data.xpos[body_id]
        to_body = lambda point: (np.asarray(point) - body_pos) @ body_rot
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, body_rot.T.flatten())
        mid = [(hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2]
        foot.add_geom(name=f"{side}_foot_collision_box", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, SOLE_HALF_THICKNESS],
                      pos=to_body(mid + [lo[2] + SOLE_HALF_THICKNESS]), quat=quat, group=3,
                      contype=ROBOT_GROUP_BIT, conaffinity=FLOOR_GROUP_BIT, friction=[1.0, 0.02, 0.01])
        foot.add_site(name=f"{side}_foot", pos=to_body(mid + [lo[2]]), quat=quat, size=[0.01, 0, 0])
        for geom in foot.geoms:
            if geom.meshname and spec.mesh(geom.meshname).file.startswith("collision/"):
                geom.contype, geom.conaffinity = 0, 0
        sole_z.append(lo[2])
    base.pos = [0, 0, -min(sole_z) + 0.001]  # soles 1 mm above the floor at the zero pose

    model = spec.compile()
    xml = spec.to_xml().replace(f'meshdir="{PACKAGE / "meshes"}"', 'meshdir="../meshes"')
    xml = re.sub(r"\n  <default>\s*<default/>\s*</default>\n", "\n", xml)  # attach_body leaves an empty default
    # MjSpec writes the actuators as <general> (gain 1, no bias), which is exactly <motor>: write them as <motor>.
    xml = re.sub(r'<general (name="\w+_motor" joint="\w+") ', r'<motor \1 gear="1" ', xml)
    assert "<general" not in xml
    header = "<!-- Generated by scripts/make_mjcf.py from urdf/full_body.urdf. Do not edit. -->\n"
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(header + xml)
    print(f"{OUTPUT}: {model.nbody} bodies, {model.njnt} joints, {model.nu} motors, "
          f"{sum(model.body_mass):.2f} kg, base at z = {base.pos[2]:.3f} m")


if __name__ == "__main__":
    main()
