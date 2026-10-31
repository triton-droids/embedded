from pathlib import Path
import json
import time

import pinocchio as pin
import numpy as np
import tsid


class WBCController:
    def __init__(self, urdf_path: Path):
        urdf_path = Path(urdf_path)
        self.model = pin.buildModelFromUrdf(
            str(urdf_path),
            pin.JointModelFreeFlyer(),
        )
        self.data = self.model.createData()

        self.robot = tsid.RobotWrapper(
            self.model,
            tsid.RootJointType.FLOATING_BASE_SYSTEM,
            False,
        )

        self.joint_names = list(self.model.names)[2:]

        metadata_path = urdf_path.with_suffix(".dynamics.json")
        self.dynamics = (
            json.loads(metadata_path.read_text()) if metadata_path.exists() else None
        )
        self.joint_damping = np.zeros(self.robot.na)
        if self.dynamics is not None:
            joints = self.dynamics["joints"]
            armature = np.array([joints[name]["armature"] for name in self.joint_names])
            self.joint_damping = np.array([
                joints[name]["damping"] for name in self.joint_names
            ])
            self.robot.set_rotor_inertias(armature)
            self.robot.set_gear_ratios(np.ones(self.robot.na))

        self.form = tsid.InverseDynamicsFormulationAccForce(
            "wbc", self.robot, False
        )
        self.solver = tsid.SolverHQuadProgFast("qp")

        self.tasks = {}
        self.contacts = {}
        self.torque_limits = None
        self.actuation_bounds = None
        self.nominal_torque_task = None
        self.last_solve_ms = 0.0
        self.contact_motion_weight = 1000.0
        self.contact_force_regularization_weight = 1e-5

        self.posture = tsid.TaskJointPosture("posture", self.robot)
        self.posture.setKp(np.full(self.robot.na, 20.0))
        self.posture.setKd(
            np.full(self.robot.na, 2 * np.sqrt(20.0))
        )

        self.add_task("posture", self.posture, weight=0.01)

        self.com_task = tsid.TaskComEquality("com", self.robot)
        self.com_task.setKp(np.full(3, 20.0))
        self.com_task.setKd(np.full(3, 2 * np.sqrt(20.0)))
        self.com_reference = None
        self.add_task("com", self.com_task, weight=1.0)

        if not self.model.existFrame("torso"):
            raise ValueError("Frame torso does not exist in the model.")

        self.body_frame_id = self.model.getFrameId("torso")
        self.body_task = tsid.TaskSE3Equality("body", self.robot, "torso")
        self.body_task.setMask(np.array([0., 0., 0., 1., 1., 1.]))
        self.body_task.setKp(np.full(6, 20.0))
        self.body_task.setKd(np.full(6, 2 * np.sqrt(20.0)))
        self.body_reference = None
        self.add_task("body", self.body_task, weight=1.0)

    def _resize_solver(self):
        self.solver.resize(
            self.form.nVar,
            self.form.nEq,
            self.form.nIn,
        )

    def set_torque_limits(self, limits):
        limits = np.asarray(limits, dtype=float)
        if limits.shape != (self.robot.na,) or not np.isfinite(limits).all() or np.any(limits <= 0):
            raise ValueError("Torque limits must be a finite positive joint vector.")
        self.torque_limits = limits.copy()
        if self.actuation_bounds is None:
            self.actuation_bounds = tsid.TaskActuationBounds("torque_bounds", self.robot)
            self.actuation_bounds.setBounds(-limits, limits)
            self.form.addActuationTask(self.actuation_bounds, 1.0, 0, 0.0)
            self._resize_solver()

    def update_contacts(self, active_frames, q):
        for frame in ("left_foot", "right_foot"):
            name = frame + "_contact"
            if frame in active_frames and name not in self.contacts:
                self.add_foot_contact(frame, q, motion_weight=self.contact_motion_weight)
            elif frame not in active_frames and name in self.contacts:
                self.remove_contact(name)

    def state_from_sim(
        self,
        base_position,
        base_quaternion_xyzw,
        base_linear_velocity,
        base_angular_velocity,
        joint_positions,
        joint_velocities,
    ):
        if self.dynamics is None:
            raise ValueError("Simulation dynamics metadata is required.")

        values = []
        for name, value, size in [
            ("base_position", base_position, 3),
            ("base_quaternion_xyzw", base_quaternion_xyzw, 4),
            ("base_linear_velocity", base_linear_velocity, 3),
            ("base_angular_velocity", base_angular_velocity, 3),
            ("joint_positions", joint_positions, self.robot.na),
            ("joint_velocities", joint_velocities, self.robot.na),
        ]:
            array = np.asarray(value, dtype=np.float64)
            if array.shape != (size,) or not np.isfinite(array).all():
                raise ValueError(f"{name} must be a finite ({size},) array.")
            values.append(array)

        position, quaternion, linear, angular, joints, velocities = values
        if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-6, rtol=0.0):
            raise ValueError("Base quaternion must have unit norm.")

        base = pin.SE3(pin.Quaternion(quaternion).matrix(), position)
        transform = self.dynamics["mujoco_base_to_pin_root"]
        offset = pin.SE3(
            np.array(transform["rotation"]), np.array(transform["translation"])
        )
        root = base * offset
        root_linear = linear + np.cross(angular, base.rotation @ offset.translation)
        q = np.concatenate([
            root.translation, pin.Quaternion(root.rotation).coeffs(), joints
        ])
        dq = np.concatenate([
            root.rotation.T @ root_linear, root.rotation.T @ angular, velocities
        ])
        return q, dq

    def add_foot_contact(self, frame_name, q, **kwargs):
        if self.dynamics is None:
            raise ValueError("Simulation dynamics metadata is required.")

        points = self.dynamics["contact_points"][frame_name]
        self.add_contact(frame_name + "_contact", frame_name, points, q, **kwargs)

    def add_task(self, name: str, task, weight: float = 1.0):
        if name in self.tasks:
            raise ValueError(f"Task {name} already exists.")

        self.form.addMotionTask(task, float(weight), 1, 0.0)
        self.tasks[name] = task
        self._resize_solver()

    def set_com_reference(self, position, velocity=None, acceleration=None):
        values = []

        for name, value in [
            ("position", position),
            ("velocity", np.zeros(3) if velocity is None else velocity),
            ("acceleration", np.zeros(3) if acceleration is None else acceleration),
        ]:
            array = np.asarray(value, dtype=np.float64)

            if array.shape != (3,) or not np.isfinite(array).all():
                raise ValueError(
                    f"CoM {name} must be a finite array of shape (3,)."
                )

            values.append(array)

        reference = tsid.TrajectorySample(3)
        reference.value(values[0])
        reference.derivative(values[1])
        reference.second_derivative(values[2])
        self.com_task.setReference(reference)
        self.com_reference = reference

    def set_body_reference(self, rotation):
        rotation = np.asarray(rotation, dtype=np.float64)

        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("Body rotation must be a finite (3, 3) array.")

        if not np.allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.0
        ) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0.0):
            raise ValueError("Body rotation must be a proper rotation matrix.")

        pose = pin.SE3(rotation.copy(), np.zeros(3))
        trajectory = tsid.TrajectorySE3Constant("body_reference", pose)
        reference = trajectory.computeNext()
        self.body_task.setReference(reference)
        self.body_reference = reference

    def add_contact(
        self,
        name: str,
        frame_name: str,
        points,
        q,
        mu: float = 0.5,
        f_min: float = 0.0,
        f_max: float = 500.0,
        motion_weight: float = 1000.0,
        motion_priority: int = 1,
    ):
        if name in self.contacts:
            raise ValueError(f"Contact {name} already exists.")

        if not self.model.existFrame(frame_name):
            raise ValueError(
                f"Frame {frame_name} does not exist in the model."
            )

        points = np.asarray(points, dtype=np.float64)

        if points.shape != (3, 4) or not np.isfinite(points).all():
            raise ValueError(
                "points must be a finite array of shape (3, 4)."
            )

        q = np.asarray(q, dtype=np.float64)

        if q.shape != (self.model.nq,) or not np.isfinite(q).all():
            raise ValueError(
                f"q must be a finite array of shape ({self.model.nq},)."
            )

        if not np.isfinite([mu, f_min, f_max]).all():
            raise ValueError("Contact parameters must be finite.")

        if mu <= 0.0 or f_min < 0.0 or f_max <= f_min:
            raise ValueError(
                "Expected mu > 0 and 0 <= f_min < f_max."
            )

        if motion_priority not in (0, 1):
            raise ValueError("Contact motion priority must be 0 or 1.")
        if not np.isfinite(motion_weight) or motion_weight <= 0.0:
            raise ValueError("Contact motion weight must be finite and positive.")

        pin.framesForwardKinematics(self.model, self.data, q)

        frame_id = self.model.getFrameId(frame_name)
        foot_pose = self.data.oMf[frame_id].copy()

        normal_world = np.array([0.0, 0.0, 1.0])
        normal_local = foot_pose.rotation.T @ normal_world

        contact = tsid.Contact6d(
            name,
            self.robot,
            frame_name,
            points,
            normal_local,
            float(mu),
            float(f_min),
            float(f_max),
        )

        contact.setKp(np.full(6, 100.0))
        contact.setKd(np.full(6, 20.0))
        contact.setReference(foot_pose)

        self.form.addRigidContact(
            contact, self.contact_force_regularization_weight, float(motion_weight), motion_priority
        )
        self.contacts[name] = contact
        self._resize_solver()

    def remove_contact(self, name: str):
        if name not in self.contacts:
            raise ValueError(f"Contact {name} does not exist.")

        if not self.form.removeRigidContact(name, 0.0):
            raise RuntimeError(f"Failed to remove contact {name}.")

        del self.contacts[name]
        self._resize_solver()

    def enable_policy_torque_tracking(self, max_correction=.25):
        if not np.isfinite(max_correction) or max_correction <= 0:
            raise ValueError("Policy torque correction must be finite and positive")
        self.max_policy_torque_correction = float(max_correction)
        if not hasattr(tsid, "TaskActuationEquality"):
            raise RuntimeError("TSID needs experimental/tsid-1.7.1-actuation-equality-binding.patch")
        self.nominal_torque_task = tsid.TaskActuationEquality("policy_torque", self.robot)
        self.form.addActuationTask(self.nominal_torque_task, 100.0, 1, 0.0)
        self.form.updateTaskWeight("posture", .001)
        self.form.updateTaskWeight("com", .01)
        self.form.updateTaskWeight("body", .01)
        self.contact_motion_weight = .1
        self.contact_force_regularization_weight = 1e-3
        self._resize_solver()

    def compute(self, q, dq, q_ref, t: float = 0.0, torque_ref=None):
        values = []

        for name, value, size in [
            ("q", q, self.model.nq),
            ("dq", dq, self.model.nv),
            ("q_ref", q_ref, self.robot.na),
        ]:
            array = np.asarray(value, dtype=np.float64)

            if array.shape != (size,) or not np.isfinite(array).all():
                raise ValueError(
                    f"{name} must be a finite array of shape "
                    f"({size},), got {array.shape}."
                )

            values.append(array)

        q, dq, q_ref = values

        if self.com_reference is None:
            position = pin.centerOfMass(self.model, self.data, q).copy()
            self.set_com_reference(position)

        if self.body_reference is None:
            pin.framesForwardKinematics(self.model, self.data, q)
            rotation = self.data.oMf[self.body_frame_id].rotation.copy()
            self.set_body_reference(rotation)

        reference = tsid.TrajectoryEuclidianConstant(
            "posture_reference", q_ref
        )
        self.posture.setReference(reference.computeNext())

        nominal = None
        if self.nominal_torque_task is not None:
            nominal = np.asarray(torque_ref, dtype=float)
            if nominal.shape != (self.robot.na,) or not np.isfinite(nominal).all():
                raise ValueError("Nominal torque must be a finite actuated joint vector")
            if self.torque_limits is None:
                raise ValueError("Policy torque tracking requires actuator torque limits")
            nominal = np.clip(nominal, -self.torque_limits, self.torque_limits)
            self.nominal_torque_task.setReference(nominal - self.joint_damping * dq[6:])

        if self.actuation_bounds is not None:
            compensation = self.joint_damping * dq[6:]
            lower, upper = -self.torque_limits, self.torque_limits
            if nominal is not None:
                lower = np.maximum(lower, nominal - self.max_policy_torque_correction)
                upper = np.minimum(upper, nominal + self.max_policy_torque_correction)
            self.actuation_bounds.setBounds(lower - compensation, upper - compensation)

        problem = self.form.computeProblemData(float(t), q, dq)
        solve_start = time.monotonic()
        solution = self.solver.solve(problem)
        self.last_solve_ms = (time.monotonic() - solve_start) * 1000.0

        if int(solution.status) != 0:
            raise RuntimeError(
                f"QP solver failed with status {solution.status}."
            )

        torques = np.array(
            self.form.getActuatorForces(solution),
            copy=True,
        )
        torques += self.joint_damping * dq[6:]

        if not np.isfinite(torques).all():
            raise RuntimeError(
                "Computed torques contain non-finite values."
            )

        if self.actuation_bounds is not None:
            error = max(float(np.max(lower - torques)), float(np.max(torques - upper)))
            if error > 1e-3:
                raise RuntimeError(f"QP violated torque bounds by {error:.6g} Nm")
            torques = np.clip(torques, lower, upper)

        return torques
