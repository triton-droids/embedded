"""Process entry point: build the world and run every simulated peripheral.

All simulated peripherals share one MjData, so they run as separate ROS nodes
inside one process (one physics thread, one executor). Each node keeps its own
name and parameters, so ``--params-file`` sections map one-to-one to nodes:

  mujoco_sim         world construction, /sim/* services, ground truth
  sim_motor_backend  replaces python_can_node
  sim_body_imu       replaces the body IMU MCU + micro-ROS Agent
  sim_head_mcu       replaces the head MCU + micro-ROS Agent
  sim_camera         replaces realsense2_camera_node (optional)
"""
from __future__ import annotations

import json
import os
import signal
import threading
import time

from ament_index_python.packages import get_package_share_directory
import mujoco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml

from . import model_builder as mb
from .motor_model import MotorBank, limits_from_registry
from .world import HeadServoConfig, SimWorld, TargetMotion

MODEL_DIR = 'models/20260914_170827'
# sim.launch.py consumes this stdout marker independently of ROS log severity.
READY_MESSAGE = 'Simulation ready'


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def load_registry(path: str) -> dict:
    with open(path) as stream:
        registry = yaml.safe_load(stream)
    return registry.get('motor_control_node', {}).get('ros__parameters', registry)


class SimControlNode(Node):
    """Owns world parameters and exposes simulator-only controls.

    Services:
      /sim/reset      std_srvs/Trigger  spawn pose, motors disabled, base held
      /sim/hold_base  std_srvs/SetBool  engage (true) or release (false) the gantry
    Topics:
      /sim/ground_truth/odom  nav_msgs/Odometry  base pose/velocity (world frame)
      /sim/status             std_msgs/String    JSON timing and state, 1 Hz
    """

    def __init__(self) -> None:
        super().__init__('mujoco_sim')
        defaults = {
            'motor_config_file': '',
            'robot_xml': '',
            'mesh_dir': '',
            'timestep': 0.0,
            'foot_clearance_m': 0.0,
            'settle_s': 2.0,
            'start_base_held': True,
            'command_timeout_s': 0.5,
            'pause_on_control_fault': True,
            'velocity_mode_kd': 2.0,
            'enable_head': True,
            'head_mass_kg': 0.05,
            'camera_fovy_deg': 42.5,
            'camera_width': 640,
            'camera_height': 480,
            'enable_target': True,
            'target_image': '',
            'target_distance_m': 1.5,
            'target_amplitude_m': 0.4,
            'target_period_s': 8.0,
            'ground_truth_rate_hz': 50.0,
            'allow_torque_only_motion': False,
            'publish_feet_ground_truth': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}

        registry_path = p['motor_config_file'] or _share('motor_control_hybrid', 'config', 'motors.yaml')
        self.registry = load_registry(registry_path)
        limits = limits_from_registry(self.registry)
        options = mb.WorldOptions(
            robot_xml=p['robot_xml'] or _share('motor_control_hybrid', MODEL_DIR,
                                               'chrobot_16kg_candidate.xml'),
            mesh_dir=p['mesh_dir'] or _share('humanoid_leg_description', 'meshes', 'robot_meshes'),
            joint_torque_limits={name: lim.max_torque for name, lim in limits.items()},
            timestep=float(p['timestep']),
            foot_clearance_m=float(p['foot_clearance_m']),
            add_head=bool(p['enable_head']),
            head_mass_kg=float(p['head_mass_kg']),
            camera_fovy_deg=float(p['camera_fovy_deg']),
            camera_resolution=(int(p['camera_width']), int(p['camera_height'])),
            add_target=bool(p['enable_target']),
            target_image=str(p['target_image']),
            target_distance_m=float(p['target_distance_m']),
        )
        model = mb.build_model(options)
        motors = MotorBank(limits, command_timeout_s=float(p['command_timeout_s']),
                           velocity_mode_kd=float(p['velocity_mode_kd']),
                           allow_torque_only_motion=bool(p['allow_torque_only_motion']))
        self.world = SimWorld(
            model, motors,
            head=HeadServoConfig() if options.add_head else None,
            target_motion=TargetMotion(float(p['target_amplitude_m']), float(p['target_period_s'])),
            base_held=bool(p['start_base_held']), settle_s=float(p['settle_s']),
            pause_on_control_fault=bool(p['pause_on_control_fault']))
        self.get_logger().warn(
            f'MuJoCo {mujoco.__version__} simulation: {model.nu} actuators, '
            f'timestep {model.opt.timestep * 1e3:.1f} ms, registry {registry_path}')

        self.odom_pub = self.create_publisher(Odometry, '/sim/ground_truth/odom', 10)
        self.feet_pub = (self.create_publisher(JointState, '/sim/ground_truth/feet', 10)
                         if p['publish_feet_ground_truth'] else None)
        self.status_pub = self.create_publisher(String, '/sim/status', 1)
        self.create_service(Trigger, '/sim/reset', self._reset)
        self.create_service(SetBool, '/sim/hold_base', self._hold_base)
        self.create_timer(1.0 / float(p['ground_truth_rate_hz']), self._publish_odom)
        self.create_timer(1.0, self._publish_status)
        self._last_status = (time.monotonic(), 0.0)

    def _reset(self, request, response):
        self.world.reset(base_held=True)
        response.success = True
        response.message = 'Simulation reset: spawn pose, base held, all motors disabled'
        self.get_logger().info(response.message)
        return response

    def _hold_base(self, request, response):
        with self.world.lock:
            if not request.data and (self.world.control_fault is not None or
                    not self.world.motors.motion_ready(time.monotonic(), 0.04)):
                response.success = False
                response.message = ('Release denied: all motors need fresh motion commands; '
                                    f'enabled={[j for j in self.world.joint_names if self.world.motors.is_enabled(j)]}')
                return response
            self.world.set_base_held(request.data)
        self._publish_status()
        response.success = True
        response.message = 'Base held' if request.data else 'Base released'
        self.get_logger().info(response.message)
        return response

    def _publish_odom(self) -> None:
        pos, quat, lin, ang = self.world.base_state()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = mb.BASE_BODY
        pose = msg.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, pos)
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, quat)
        # Odometry twist is nominally child-frame; this ground truth is world-frame, see docstring.
        twist = msg.twist.twist
        twist.linear.x, twist.linear.y, twist.linear.z = map(float, lin)
        twist.angular.x, twist.angular.y, twist.angular.z = map(float, ang)
        self.odom_pub.publish(msg)
        if self.feet_pub is not None:
            heights, forces = self.world.foot_contacts()
            feet = JointState()
            feet.header = msg.header
            feet.name = ['left_foot', 'right_foot']
            feet.position = heights.tolist()
            feet.effort = forces.tolist()
            self.feet_pub.publish(feet)

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self.world.lock:
            sim_time = float(self.world.data.time)
        last_wall, last_sim = self._last_status
        self._last_status = (now, sim_time)
        rtf = (sim_time - last_sim) / (now - last_wall) if now > last_wall else 0.0
        self.status_pub.publish(String(data=json.dumps(dict(
            sim_time_s=sim_time, real_time_factor=round(rtf, 3),
            physics_overruns=self.world.overruns, base_held=self.world.base_held(),
            control_fault=self.world.control_fault, physics_paused=self.world.control_fault is not None,
            estop_active=self.world.motors.estop_active))))


def _open_viewer(world: SimWorld):
    """Finish GUI initialization before advertising simulator readiness."""
    from .viewer import ViewerProcess

    viewer = ViewerProcess(world)
    return viewer, viewer


def _run_viewer(mirror, viewer) -> None:
    while viewer.is_running() and rclpy.ok():
        mirror.update()
        time.sleep(1.0 / 60.0)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    control = SimControlNode()
    world = control.world

    # Imported here so a missing optional dependency only affects its own node.
    from .camera_node import SimCameraNode
    from .imu_nodes import SimBodyImuNode, SimHeadMcuNode
    from .motor_backend_node import SimMotorBackendNode

    control.declare_parameter('enable_camera', False)
    control.declare_parameter('use_viewer', False)
    nodes = [control, SimMotorBackendNode(world, control.registry), SimBodyImuNode(world)]
    if world.head is not None:
        nodes.append(SimHeadMcuNode(world))
        if control.get_parameter('enable_camera').value:
            nodes.append(SimCameraNode(world))

    mirror = viewer = None
    if control.get_parameter('use_viewer').value:
        try:
            mirror, viewer = _open_viewer(world)
        except Exception as exc:  # No display: keep running headless.
            control.get_logger().error(f'MuJoCo viewer unavailable, running headless: {exc}')

    # The minimal loop has short callbacks; avoid a Python worker pool and
    # its extra GIL/dispatch contention. Physics remains on its own thread.
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    world.start()
    spin = threading.Thread(target=executor.spin, name='ros_executor', daemon=True)
    spin.start()
    # Launch captures stdout in its log and uses this marker to start the stack.
    # Keep readiness independent of the diagnostics severity (WARN by default).
    # Threads being alive is not proof that the executor has published sensors.
    ready_until = time.monotonic() + 5.0
    while rclpy.ok() and spin.is_alive() and time.monotonic() < ready_until:
        if nodes[1].first_feedback.is_set() and nodes[2].first_sample.is_set():
            break
        time.sleep(0.01)
    else:
        executor.shutdown()
        spin.join(timeout=2.0)
        world.stop()
        if viewer is not None:
            viewer.close()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise RuntimeError('Simulator failed to publish initial IMU/joint feedback')
    print(READY_MESSAGE, flush=True)
    try:
        if viewer is not None:
            try:
                _run_viewer(mirror, viewer)
            except Exception as exc:
                control.get_logger().error(f'MuJoCo viewer unavailable, running headless: {exc}')
        while rclpy.ok() and spin.is_alive():
            spin.join(timeout=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        world.stop()
        if viewer is not None:
            viewer.close()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
