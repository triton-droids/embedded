from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "humanoid_leg_description/urdf/human_offset_corrected.urdf"
MJCF = ROOT / "humanoid_control/motor_control_hybrid/models/20260914_170827/chrobot_16kg_candidate.xml"
MESHES = ROOT / "humanoid_leg_description/meshes/robot_meshes"


def sync():
    spec = mujoco.MjSpec.from_file(str(MJCF))
    spec.meshdir = str(MESHES)
    mj_model = spec.compile()
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    model = pin.buildModelFromUrdf(str(URDF), pin.JointModelFreeFlyer())
    data = model.createData()
    q = pin.neutral(model)
    pin.framesForwardKinematics(model, data, q)

    torso = mj_model.body("torso").id
    mj_torso = pin.SE3(mj_data.xmat[torso].reshape(3, 3), mj_data.xpos[torso])
    root_pose = mj_torso * data.oMf[model.getFrameId("torso")].inverse()
    base = mj_model.body("floating_base").id
    mj_base = pin.SE3(mj_data.xmat[base].reshape(3, 3), mj_data.xpos[base])
    base_to_root = mj_base.inverse() * root_pose
    q[:3] = root_pose.translation
    q[3:7] = pin.Quaternion(root_pose.rotation).coeffs()
    pin.framesForwardKinematics(model, data, q)

    text = URDF.read_text()
    tree = ET.fromstring(text)
    for link in tree.findall("link"):
        name = link.attrib["name"]
        if link.find("inertial") is None:
            continue
        body = mj_model.body(name).id
        frame = data.oMf[model.getFrameId(name)]
        com = frame.actInv(mj_data.xipos[body])
        rotation = frame.rotation.T @ mj_data.ximat[body].reshape(3, 3)
        inertia = rotation @ np.diag(mj_model.body_inertia[body]) @ rotation.T
        xyz = " ".join(f"{x:.16g}" for x in com)
        attrs = " ".join(
            f'{key}="{inertia[i, j]:.16g}"'
            for key, i, j in [
                ("ixx", 0, 0), ("iyy", 1, 1), ("izz", 2, 2),
                ("ixy", 0, 1), ("ixz", 0, 2), ("iyz", 1, 2),
            ]
        )
        inertial = (
            f'<inertial>\n      <origin xyz="{xyz}" rpy="0 0 0"/>\n'
            f'      <mass value="{mj_model.body_mass[body]:.16g}"/>\n'
            f'      <inertia {attrs}/>\n    </inertial>'
        )
        pattern = rf'(<link name="{re.escape(name)}">\s*)<inertial>.*?</inertial>'
        text, count = re.subn(pattern, lambda match: match[1] + inertial, text, count=1, flags=re.S)
        if count != 1:
            raise ValueError(f"Cannot replace inertial for {name}")

    contacts = {}
    for side in ("left", "right"):
        frame_name = side + "_foot"
        frame = data.oMf[model.getFrameId(frame_name)]
        geom = mj_model.geom(side + "_foot_collision_box").id
        half = mj_model.geom_size[geom]
        rotation = mj_data.geom_xmat[geom].reshape(3, 3)
        points = [
            frame.actInv(mj_data.geom_xpos[geom] + rotation @ np.array([x * half[0], y * half[1], -half[2]]))
            for x, y in [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        ]
        contacts[frame_name] = np.column_stack(points).tolist()

    metadata = {
        "source": str(MJCF.relative_to(ROOT)),
        "source_sha256": hashlib.sha256(MJCF.read_bytes()).hexdigest(),
        "mujoco_base_to_pin_root": {
            "translation": base_to_root.translation.tolist(),
            "rotation": base_to_root.rotation.tolist(),
        },
        "joints": {
            name: {
                "armature": float(mj_model.dof_armature[mj_model.joint(name).dofadr[0]]),
                "damping": float(mj_model.dof_damping[mj_model.joint(name).dofadr[0]]),
            }
            for name in list(model.names)[2:]
        },
        "contact_points": contacts,
    }
    URDF.write_text(text)
    URDF.with_suffix(".dynamics.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    sync()
