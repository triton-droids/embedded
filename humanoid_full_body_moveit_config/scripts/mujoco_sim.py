#!/usr/bin/env python3
"""MuJoCo backend for the full-body MoveIt inspection: joint states out, trajectories in.

- Loads humanoid_full_body_description/urdf/<urdf> (default full_body.urdf) into MuJoCo. torso_link is welded
  to the world (fixed base, like the MoveIt model).
- Copies the SRDF <disable_collisions> pairs into MuJoCo <exclude> pairs. Without them the
  CAD meshes of neighbouring links touch at the zero pose and push each other apart.
- Gives every joint a position actuator (PD servo). By default it applies gravity compensation,
  so the servos only have to track the trajectory.
- Publishes /joint_states. Each controller in config/moveit_controllers.yaml gets a
  control_msgs/FollowJointTrajectory action server: <controller>/follow_joint_trajectory.

Needs the `mujoco` Python module: source scripts/setup_rosenv.sh first.
Time is wall-clock: physics runs in real time, and /clock is not published.
"""

import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
from control_msgs.action import FollowJointTrajectory
try:
    import mujoco
except ImportError as error:
    raise SystemExit("mujoco_sim.py needs the mujoco Python module: run "
                     "`source scripts/setup_rosenv.sh` in the repository root first") from error
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

DESCRIPTION = "humanoid_full_body_description"
CONFIG = "humanoid_full_body_moveit_config"


def build_model(kp, kv, gravity_compensation, urdf_name="full_body.urdf"):
    share = str(get_package_share_path(DESCRIPTION))
    robot = ET.parse(f"{share}/urdf/{urdf_name}").getroot()
    for mesh in robot.iter("mesh"):
        mesh.set("filename", mesh.get("filename").replace(f"package://{DESCRIPTION}", share))
    compiler = ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler")
    # Keep every link as its own body and keep the visual meshes for the viewer.
    compiler.attrib.update(strippath="false", fusestatic="false", discardvisual="false",
                           balanceinertia="true")
    spec = mujoco.MjSpec.from_string(ET.tostring(robot, encoding="unicode"))

    srdf = ET.parse(get_package_share_path(CONFIG) / "config/full_body.srdf").getroot()
    for pair in srdf.findall("disable_collisions"):
        exclude = spec.add_exclude()
        exclude.bodyname1, exclude.bodyname2 = pair.get("link1"), pair.get("link2")

    # Some distal links have about 2e-3 kg*m^2 about their joint. An explicit integrator
    # with the servo damping below diverges within a few steps, so treat damping
    # implicitly and add a rotor-like armature.
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if gravity_compensation:
        for body in spec.bodies[1:]:
            body.gravcomp = 1.0
    for joint in spec.joints:
        joint.damping = 0.5  # N*m*s/rad
        joint.armature = 0.01  # kg*m^2, placeholder reflected rotor inertia
        actuator = spec.add_actuator(name=joint.name, target=joint.name,
                                     trntype=mujoco.mjtTrn.mjTRN_JOINT)
        actuator.set_to_position(kp=kp, kv=kv)

    # A floor below the feet, for the viewer only: no contacts, and the robot hangs fixed.
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    floor_z = float(np.min(data.geom_xpos[:, 2])) - 0.15
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1],
                            pos=[0, 0, floor_z], rgba=[0.35, 0.38, 0.42, 1],
                            contype=0, conaffinity=0)
    spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    return spec.compile()


class Trajectory:
    """Joint-space interpolation of a JointTrajectory: cubic Hermite when velocities are given."""

    def __init__(self, msg):
        self.times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                               for p in msg.points])
        self.q = np.array([p.positions for p in msg.points])
        has_velocity = all(len(p.velocities) == len(msg.joint_names) for p in msg.points)
        self.dq = np.array([p.velocities for p in msg.points]) if has_velocity else None
        self.duration = float(self.times[-1])

    def sample(self, t):
        if t <= self.times[0]:
            return self.q[0]
        if t >= self.duration:
            return self.q[-1]
        i = int(np.searchsorted(self.times, t)) - 1
        h = self.times[i + 1] - self.times[i]
        if h <= 0.0:
            return self.q[i + 1]
        s = (t - self.times[i]) / h
        if self.dq is None:
            return (1 - s) * self.q[i] + s * self.q[i + 1]
        h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
        h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
        return (h00 * self.q[i] + h10 * h * self.dq[i]
                + h01 * self.q[i + 1] + h11 * h * self.dq[i + 1])


class MujocoSim(Node):
    def __init__(self):
        super().__init__("full_body_mujoco_sim")
        kp = self.declare_parameter("kp", 200.0).value  # N*m/rad
        kv = self.declare_parameter("kv", 8.0).value  # N*m*s/rad
        gravity_compensation = self.declare_parameter("gravity_compensation", True).value
        self.goal_tolerance = self.declare_parameter("goal_tolerance", 0.02).value  # rad
        self.settle_time = self.declare_parameter("settle_time", 1.0).value  # s
        publish_rate = self.declare_parameter("publish_rate", 100.0).value  # Hz
        self.use_viewer = self.declare_parameter("viewer", True).value
        urdf_name = self.declare_parameter("urdf", "full_body.urdf").value  # file in the description's urdf/

        started = time.monotonic()
        self.model = build_model(kp, kv, gravity_compensation, urdf_name)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.get_logger().info(
            f"MuJoCo model: {self.model.njnt} joints, {self.model.ngeom} geoms, "
            f"dt {self.model.opt.timestep} s, built in {time.monotonic() - started:.1f} s, "
            f"{self.data.ncon} contacts at the zero pose")

        self.joint_names = [self.model.joint(j).name for j in range(self.model.njnt)]
        self.qpos_index = {n: int(self.model.joint(n).qposadr[0]) for n in self.joint_names}
        self.dof_index = {n: int(self.model.joint(n).dofadr[0]) for n in self.joint_names}
        self.actuator_index = {n: self.model.actuator(n).id for n in self.joint_names}
        self.lock = threading.Lock()
        self.busy_joints = set()

        self.state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / publish_rate, self.publish_state)

        controllers = yaml.safe_load(
            (get_package_share_path(CONFIG) / "config/moveit_controllers.yaml").read_text())
        manager = controllers["moveit_simple_controller_manager"]
        group = ReentrantCallbackGroup()
        self.servers = []
        for name in manager["controller_names"]:
            joints = manager[name]["joints"]
            self.servers.append(ActionServer(
                self, FollowJointTrajectory, f"{name}/{manager[name]['action_ns']}",
                execute_callback=lambda goal, j=joints: self.execute(goal, j),
                goal_callback=lambda request, j=joints: self.accept(request, j),
                cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                callback_group=group))
            self.get_logger().info(f"{name}/follow_joint_trajectory: {', '.join(joints)}")

        self.running = True
        self.physics = threading.Thread(target=self.physics_loop, daemon=True)
        self.physics.start()

    def physics_loop(self):
        viewer = None
        if self.use_viewer:
            from mujoco import viewer as mujoco_viewer  # needs a display; import only when used
            viewer = mujoco_viewer.launch_passive(self.model, self.data)
            viewer.cam.lookat[:] = [0, 0, -0.4]
            viewer.cam.distance = 2.5
        dt = self.model.opt.timestep
        next_step = time.monotonic()
        last_sync = 0.0
        while self.running and rclpy.ok():
            with self.lock:
                mujoco.mj_step(self.model, self.data)
            now = time.monotonic()
            if viewer is not None:
                if not viewer.is_running():
                    self.get_logger().info("Viewer closed, stopping the simulation")
                    break
                if now - last_sync > 1 / 60:
                    with self.lock:
                        viewer.sync()
                    last_sync = now
            next_step += dt
            delay = next_step - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:  # far behind wall clock: skip ahead, do not try to catch up
                next_step = time.monotonic()
        self.running = False
        if viewer is not None:
            viewer.close()

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        with self.lock:
            msg.position = [float(self.data.qpos[self.qpos_index[n]]) for n in self.joint_names]
            msg.velocity = [float(self.data.qvel[self.dof_index[n]]) for n in self.joint_names]
            msg.effort = [float(self.data.actuator_force[self.actuator_index[n]])
                          for n in self.joint_names]
        self.state_pub.publish(msg)

    def accept(self, request, controller_joints):
        trajectory = request.trajectory
        unknown = set(trajectory.joint_names) - set(controller_joints)
        if not trajectory.points or unknown:
            self.get_logger().error(f"Rejecting trajectory: no points or unknown joints {unknown}")
            return GoalResponse.REJECT
        if self.busy_joints & set(trajectory.joint_names):
            self.get_logger().error("Rejecting trajectory: joints already executing another goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def set_targets(self, names, positions):
        with self.lock:
            for name, position in zip(names, positions):
                self.data.ctrl[self.actuator_index[name]] = position

    def positions(self, names):
        with self.lock:
            return np.array([self.data.qpos[self.qpos_index[n]] for n in names])

    def execute(self, goal_handle, _controller_joints):
        request = goal_handle.request
        names = list(request.trajectory.joint_names)
        trajectory = Trajectory(request.trajectory)
        # Continuous joints: MoveIt may express the trajectory in a different 2*pi turn
        # than MuJoCo's unbounded qpos. Shift each to the turn nearest the current angle.
        current = self.positions(names)
        for k, name in enumerate(names):
            if not self.model.jnt_limited[self.model.joint(name).id]:
                turns = np.round((current[k] - trajectory.q[0, k]) / (2 * np.pi))
                trajectory.q[:, k] += turns * 2 * np.pi
        result = FollowJointTrajectory.Result()
        self.busy_joints |= set(names)
        try:
            started = time.monotonic()
            feedback = FollowJointTrajectory.Feedback(joint_names=names)
            while True:
                t = time.monotonic() - started
                if goal_handle.is_cancel_requested:
                    self.set_targets(names, self.positions(names))  # hold where it is
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "canceled"
                    return result
                if not self.running:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = "simulation stopped"
                    return result
                target = trajectory.sample(t)
                self.set_targets(names, target)
                error = np.abs(self.positions(names) - trajectory.q[-1])
                if t >= trajectory.duration and (error.max() <= self.goal_tolerance
                                                 or t >= trajectory.duration + self.settle_time):
                    break
                feedback.desired.positions = [float(x) for x in target]
                feedback.actual.positions = [float(x) for x in self.positions(names)]
                goal_handle.publish_feedback(feedback)
                time.sleep(0.01)
            if error.max() > self.goal_tolerance:
                worst = names[int(np.argmax(error))]
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f"{worst} is {error.max():.3f} rad from the goal"
                self.get_logger().warn(result.error_string)
                return result
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        finally:
            self.busy_joints -= set(names)


def main():
    rclpy.init()
    node = MujocoSim()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        while rclpy.ok() and node.running:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.physics.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
