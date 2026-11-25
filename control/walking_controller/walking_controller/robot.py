"""Pinocchio robot model aligned with the MuJoCo simulation, plus registry limits.

No ROS imports: shared by the offline harness, the controllers and the tests.
Adds one frame per sole corner so a foot can also be held on one edge (line
contact) instead of flat.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

FEET = ('left_foot', 'right_foot')
TORSO = 'torso'
# Peak torque per motor model when motors.yaml leaves max_torque_nm empty.
PEAK_TORQUE_NM = {'rs-02': 17.0, 'rs-03': 60.0, 'rs-04': 120.0}


def other(foot: str) -> str:
    return FEET[1] if foot == FEET[0] else FEET[0]


@dataclass(frozen=True)
class JointLimits:
    torque: np.ndarray
    lower: np.ndarray
    upper: np.ndarray


def load_limits(registry_path, names) -> JointLimits:
    motors = yaml.safe_load(Path(registry_path).read_text())['motor_control_node']['ros__parameters']['motors']
    torque = [float(m['max_torque_nm']) if m.get('max_torque_nm') is not None
              else PEAK_TORQUE_NM[m['model']] for m in (motors[n] for n in names)]
    return JointLimits(np.array(torque),
                       np.array([float(motors[n]['min_position']) for n in names]),
                       np.array([float(motors[n]['max_position']) for n in names]))


class RobotModel:
    """Floating-base model; q/v follow Pinocchio (local base twist)."""

    def __init__(self, urdf_path):
        urdf_path = Path(urdf_path)
        self.model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
        self.dynamics = json.loads(urdf_path.with_suffix('.dynamics.json').read_text())
        joints = self.dynamics['joints']
        self.joint_names = list(self.model.names)[2:]
        self.na = len(self.joint_names)
        self.armature = np.array([joints[n]['armature'] for n in self.joint_names])
        self.damping = np.array([joints[n]['damping'] for n in self.joint_names])
        t = self.dynamics['mujoco_base_to_pin_root']
        self.base_offset = pin.SE3(np.array(t['rotation']), np.array(t['translation']))
        self.contact_points = {f: np.array(self.dynamics['contact_points'][f]) for f in FEET}
        # One frame per sole corner, named <foot>_c<i>.
        self.corner_frames = {}
        for foot in FEET:
            parent = self.model.frames[self.model.getFrameId(foot)]
            names = []
            for i, point in enumerate(self.contact_points[foot].T):
                name = f'{foot}_c{i}'
                placement = parent.placement * pin.SE3(np.eye(3), point)
                self.model.addFrame(pin.Frame(name, parent.parent, self.model.getFrameId(foot),
                                              placement, pin.FrameType.OP_FRAME))
                names.append(name)
            self.corner_frames[foot] = names
        self.data = self.model.createData()
        self.frame_id = {f: self.model.getFrameId(f) for f in FEET + (TORSO,)}
        self.mass = float(sum(i.mass for i in self.model.inertias))

    def state_from_sim(self, base_position, base_quat_xyzw, base_linear_world,
                       base_angular_world, joint_positions, joint_velocities):
        """MuJoCo floating base (world-frame twist) to Pinocchio (q, v)."""
        quat = np.asarray(base_quat_xyzw, dtype=float)
        position, linear, angular = (np.asarray(x, dtype=float) for x in
                                     (base_position, base_linear_world, base_angular_world))
        base = pin.SE3(pin.Quaternion(quat / np.linalg.norm(quat)).matrix(), position)
        root = base * self.base_offset
        root_linear = linear + np.cross(angular, base.rotation @ self.base_offset.translation)
        q = np.concatenate([root.translation, pin.Quaternion(root.rotation).coeffs(), joint_positions])
        v = np.concatenate([root.rotation.T @ root_linear, root.rotation.T @ angular, joint_velocities])
        return q, v

    def forward(self, q, v=None):
        v = np.zeros(self.model.nv) if v is None else v
        pin.forwardKinematics(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)
        pin.centerOfMass(self.model, self.data, q, v)

    @property
    def com(self):
        return self.data.com[0]

    @property
    def vcom(self):
        return self.data.vcom[0]

    def frame(self, name) -> pin.SE3:
        return self.data.oMf[self.frame_id[name]]

    def sole_points(self, foot) -> np.ndarray:
        """World corner positions (3, 4) after forward()."""
        pose = self.frame(foot)
        return pose.rotation @ self.contact_points[foot] + pose.translation[:, None]

    def sole_center(self, foot) -> np.ndarray:
        return self.sole_points(foot).mean(axis=1)

    def inner_corners(self, foot) -> list[int]:
        """Indices of the two corners on the edge facing the other foot."""
        points = self.sole_points(foot)[:2]
        toward = self.sole_center(other(foot))[:2] - self.sole_center(foot)[:2]
        toward /= np.linalg.norm(toward)
        order = np.argsort(-(toward @ (points - points.mean(axis=1, keepdims=True))))
        return sorted(order[:2].tolist())
