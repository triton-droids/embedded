"""Process entry point: build the world and run every simulated peripheral.

All simulated peripherals share one MjData, so they run as separate ROS nodes
inside one process (one physics thread and a sensor executor, with optional
dedicated command and body-IMU executors). Each node keeps its own
name and parameters, so ``--params-file`` sections map one-to-one to nodes:

  mujoco_sim         world construction, /sim/* services, ground truth
  sim_motor_backend  replaces python_can_node
  sim_motor_command_rx  optional isolated command/estop reception in WBC mode
  sim_body_imu       replaces the body IMU MCU + micro-ROS Agent
  sim_head_mcu       replaces the head MCU + micro-ROS Agent
  sim_camera         replaces realsense2_camera_node (optional)
"""
from __future__ import annotations
from .publish_sample import publish_sample

import gc
import json
import importlib
import os
import signal
import sys
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
from .imu_shm import ImuSnapshotWriter
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
            'dedicated_command_executor': False,
            'velocity_mode_kd': 2.0,
            'enable_head': True,
            'head_mass_kg': 0.05,
            'camera_fovy_deg': 42.5,
            'camera_width': 640,
            'camera_height': 480,
            'enable_target': True,
            'target_image': '',
            'scene_builder': '',
            'scene_nodes': '',
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
            scene_builder=str(p['scene_builder']),
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

        self.odom_pub = self.create_publisher(Odometry, '/sim/ground_truth/odom', 2)
        self.feet_pub = (self.create_publisher(JointState, '/sim/ground_truth/feet', 2)
                         if p['publish_feet_ground_truth'] else None)
        self.status_pub = self.create_publisher(String, '/sim/status', 2)
        self.create_service(Trigger, '/sim/reset', self._reset)
        self.create_service(SetBool, '/sim/hold_base', self._hold_base)
        self.create_timer(1.0 / float(p['ground_truth_rate_hz']), self._publish_odom)
        self.create_timer(1.0, self._publish_status)
        self._last_status = (time.monotonic(), 0.0)
        self._odom_msg = None
        self._feet_msg = None

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
        begin = time.monotonic()
        # Pose and contact feedback must describe the same physical step.
        with self.world.lock:
            pos, quat, lin, ang = self.world.base_state()
            stamp_ns = self.world.sample_stamp_ns
            contacts = self.world.foot_contacts() if self.feet_pub is not None else None
        # Reused across publications: this executor is single-threaded and no
        # consumer holds a reference after publish() returns.
        if self._odom_msg is None:
            self._odom_msg = Odometry()
        msg = self._odom_msg
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        msg.header.frame_id = 'world'
        msg.child_frame_id = mb.BASE_BODY
        pose = msg.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, pos)
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, quat)
        # Odometry twist is nominally child-frame; this ground truth is world-frame, see docstring.
        twist = msg.twist.twist
        twist.linear.x, twist.linear.y, twist.linear.z = map(float, lin)
        twist.angular.x, twist.angular.y, twist.angular.z = map(float, ang)
        publish_sample(self, self.odom_pub, msg, '/sim/ground_truth/odom')
        if self.feet_pub is not None:
            heights, forces = contacts
            if self._feet_msg is None:
                self._feet_msg = JointState()
            feet = self._feet_msg
            feet.header = msg.header
            feet.name = ['left_foot', 'right_foot']
            feet.position = heights.tolist()
            feet.effort = forces.tolist()
            publish_sample(self, self.feet_pub, feet, '/sim/ground_truth/feet')
        self._max_odom_callback_s = max(getattr(self, '_max_odom_callback_s', 0.0), time.monotonic()-begin)

    def _publish_status(self) -> None:
        begin = time.monotonic()
        now = time.monotonic()
        with self.world.lock:
            sim_time = float(self.world.data.time)
        last_wall, last_sim = self._last_status
        self._last_status = (now, sim_time)
        rtf = (sim_time - last_sim) / (now - last_wall) if now > last_wall else 0.0
        publish_sample(self, self.status_pub, String(data=json.dumps(dict(
            sim_time_s=sim_time, real_time_factor=round(rtf, 3),
            physics_overruns=self.world.overruns, base_held=self.world.base_held(),
            control_fault=self.world.control_fault, physics_paused=self.world.control_fault is not None,
            max_odom_callback_s=getattr(self, '_max_odom_callback_s', 0.0),
            max_status_callback_s=getattr(self, '_max_status_callback_s', 0.0),
            publish_failures=dict(getattr(self, '_publish_failures', {})),
            estop_active=self.world.motors.estop_active))), '/sim/status')
        self._max_status_callback_s = max(getattr(self, '_max_status_callback_s', 0.0), time.monotonic()-begin)


def _open_viewer(world: SimWorld):
    """Finish GUI initialization before advertising simulator readiness."""
    from .viewer import ViewerProcess

    viewer = ViewerProcess(world)
    return viewer, viewer


def _run_viewer(mirror, viewer) -> None:
    from .viewer import VIEWER_RATE_HZ

    while viewer.is_running() and rclpy.ok():
        mirror.update()
        time.sleep(1.0 / VIEWER_RATE_HZ)


def make_executors(nodes, command_node=None, dedicated_imu=True):
    """Keep body-IMU deadlines independent of slower feedback/diagnostic work.

    Each executor is single-threaded; this avoids a general worker pool while
    preventing an unrelated DDS publish/timer from serializing the IMU timer.
    It does not remove physics-lock contention, GIL or OS scheduling limits.
    """
    from .imu_nodes import SimBodyImuNode
    executor = SingleThreadedExecutor()
    imu_executor = None
    for node in nodes:
        if dedicated_imu and isinstance(node, SimBodyImuNode):
            if imu_executor is None:
                imu_executor = SingleThreadedExecutor()
            imu_executor.add_node(node)
        else:
            executor.add_node(node)
    executors = [executor]
    if imu_executor is not None:
        executors.append(imu_executor)
    if command_node is not None:
        command_executor = SingleThreadedExecutor()
        command_executor.add_node(command_node)
        executors.append(command_executor)
    return executors


def spin_executor(executor):
    from .imu_nodes import SimBodyImuNode
    from .thread_setup import configure_thread
    configure_thread('imu' if any(isinstance(n, SimBodyImuNode) for n in executor.get_nodes()) else 'other')
    executor.spin()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    control = SimControlNode()
    world = control.world
    # The physics thread steps every 5 ms. With CPython's default 5 ms GIL
    # switch interval it can wait a whole period for the executor or camera
    # thread to yield; e.g. 0.0005 bounds that wait. 0 keeps the default.
    control.declare_parameter('gil_switch_interval_s', 0.0)
    switch_interval = float(control.get_parameter('gil_switch_interval_s').value)
    if switch_interval > 0.0:
        sys.setswitchinterval(switch_interval)
    from rclpy.utilities import get_rmw_implementation_identifier
    world.rmw_implementation = get_rmw_implementation_identifier()
    # Name the active transport profile: an unset variable silently keeps stock
    # Fast DDS limits, so "did the tuning apply?" must be answerable from a log.
    profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
    control.get_logger().info(
        f"Fast DDS profile: {profile or 'none (stock limits)'}")

    # Imported here so a missing optional dependency only affects its own node.
    from .camera_node import SimCameraNode
    from .imu_nodes import SimBodyImuNode, SimHeadMcuNode
    from .motor_backend_node import SimMotorBackendNode

    control.declare_parameter('enable_camera', False)
    control.declare_parameter('use_viewer', False)
    control.declare_parameter('collection_output_dir', '')
    control.declare_parameter('collection_seed', 1)
    control.declare_parameter('collection_duration_s', 15.0)
    control.declare_parameter('collection_profile', 'standard')
    recorder = None
    if control.get_parameter('collection_output_dir').value:
        if control.get_parameter('enable_camera').value or control.get_parameter('use_viewer').value:
            raise ValueError('Collection requires camera and viewer disabled')
        from .collection import EpisodeRecorder
        recorder = EpisodeRecorder(world, control.get_parameter('collection_output_dir').value,
            seed=control.get_parameter('collection_seed').value,
            duration_s=control.get_parameter('collection_duration_s').value,
            profile=control.get_parameter('collection_profile').value)
        world.recorder = recorder
        # Preserve estop/motor disable, but let gravity/contact evolve afterwards.
        # Otherwise the diagnostic freeze censors almost every fall outcome.
        world.pause_on_control_fault = False
    control.declare_parameter('remote_viewer', False)
    control.declare_parameter('remote_viewer_host', '127.0.0.1')
    control.declare_parameter('remote_viewer_port', 8765)
    control.declare_parameter('remote_viewer_rate_hz', 15.0)
    control.declare_parameter('dedicated_imu_executor', True)
    control.declare_parameter('gc_freeze', True)
    # Publish /imu/data_raw from a separate process reading a shared segment.
    # It cannot be delayed by this process's own publishes, which is where the
    # measured 331.9 ms IMU publish stall occurred.
    control.declare_parameter('imu_split', False)
    control.declare_parameter('imu_snapshot_path', '')
    command_node = (Node('sim_motor_command_rx', start_parameter_services=False,
                         enable_rosout=False)
                    if control.get_parameter('dedicated_command_executor').value else None)
    imu_split = bool(control.get_parameter('imu_split').value)
    if recorder is not None and imu_split:
        raise ValueError('Collection records the in-process IMU publication; use imu_split:=false')
    imu_writer = None
    imu_node = None
    imu_diagnostics = (Node('sim_body_imu_diagnostics', start_parameter_services=False,
                            enable_rosout=False)
                       if control.get_parameter('dedicated_imu_executor').value and not imu_split
                       else None)
    if imu_split:
        # The physics thread writes one record per step; sim_imu_publisher reads
        # it. Nothing is published from here, so no publish call can block it.
        imu_writer = ImuSnapshotWriter(
            str(control.get_parameter('imu_snapshot_path').value) or None)
        world.imu_writer = imu_writer
    else:
        imu_node = SimBodyImuNode(world, diagnostics_node=imu_diagnostics)
    backend = SimMotorBackendNode(world, control.registry, command_node=command_node)
    nodes = [control, backend]
    if imu_node is not None:
        nodes.append(imu_node)
    if imu_diagnostics is not None:
        nodes.append(imu_diagnostics)
    scene_nodes = control.get_parameter('scene_nodes').value
    if scene_nodes:
        module, factory = scene_nodes.split(':', 1)
        nodes.extend(getattr(importlib.import_module(module), factory)(world))
    if world.head is not None:
        nodes.append(SimHeadMcuNode(world))
        if control.get_parameter('enable_camera').value:
            nodes.append(SimCameraNode(world))

    mirror = viewer = None
    remote = None
    if control.get_parameter('remote_viewer').value:
        from .remote_viewer import RemoteViewerServer
        try:
            remote = RemoteViewerServer(world,
                host=str(control.get_parameter('remote_viewer_host').value),
                port=int(control.get_parameter('remote_viewer_port').value),
                rate_hz=float(control.get_parameter('remote_viewer_rate_hz').value))
            control.get_logger().warn(f'Read-only remote viewer listening on port {remote.port}')
        except Exception as exc:
            control.get_logger().error(f'Remote viewer unavailable, continuing headless: {exc}')
    if control.get_parameter('use_viewer').value:
        try:
            mirror, viewer = _open_viewer(world)
        except Exception as exc:  # No display: keep running headless.
            control.get_logger().error(f'MuJoCo viewer unavailable, running headless: {exc}')

    # Command admission and optional body IMU run independently of feedback and
    # diagnostics. Shared motor/physics state still uses world.lock.
    executors = make_executors(nodes, command_node,
        dedicated_imu=bool(control.get_parameter('dedicated_imu_executor').value))
    if command_node is not None:
        nodes.append(command_node)
    world.start()
    spins = []
    for i, item in enumerate(executors):
        imu_thread = any(isinstance(n, SimBodyImuNode) for n in item.get_nodes())
        spins.append(threading.Thread(target=spin_executor, args=(item,),
                     name='sim_body_imu_executor' if imu_thread else f'ros_executor_{i}',
                     daemon=True))
    for spin in spins:
        spin.start()
    # Do this after spawning critical threads: unprivileged nice increases
    # cannot be undone by their children. Each thread sets its own profile.
    from .thread_setup import configure_thread
    configure_thread('other')
    # Launch captures stdout in its log and uses this marker to start the stack.
    # Keep readiness independent of the diagnostics severity (WARN by default).
    # Threads being alive is not proof that the executor has published sensors.
    ready_until = time.monotonic() + 5.0
    while rclpy.ok() and all(t.is_alive() for t in spins) and time.monotonic() < ready_until:
        imu_ready = imu_node is None or imu_node.first_sample.is_set()
        if backend.first_feedback.is_set() and imu_ready:
            break
        time.sleep(0.01)
    else:
        for item in executors:
            item.shutdown()
        for spin in spins:
            spin.join(timeout=2.0)
        world.stop()
        if imu_writer is not None:
            imu_writer.close()
        if remote is not None:
            remote.close()
        if viewer is not None:
            viewer.close()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise RuntimeError('Simulator failed to publish initial IMU/joint feedback')
    # Long-lived objects created during startup (model, registry, nodes) would
    # otherwise be rescanned by every cyclic collection. Frozen objects stop
    # participating in GC passes, which removes the measured ~20 ms pauses from
    # the physics process without disabling collection for new objects.
    if control.get_parameter('gc_freeze').value:
        gc.collect()
        gc.freeze()
    print(READY_MESSAGE, flush=True)
    try:
        if viewer is not None:
            try:
                _run_viewer(mirror, viewer)
            except Exception as exc:
                control.get_logger().error(f'MuJoCo viewer unavailable, running headless: {exc}')
        while rclpy.ok() and all(t.is_alive() for t in spins):
            if recorder is not None and recorder.poll():
                break
            spins[0].join(timeout=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for item in executors:
            item.shutdown()
        for spin in spins:
            spin.join(timeout=2.0)
        world.stop()
        if recorder is not None:
            with world.lock:
                recorder.finish('interrupted')
                metadata = recorder.save()
            print('Collection saved: ' + json.dumps(metadata), flush=True)
        if imu_writer is not None:
            imu_writer.close()
        if remote is not None:
            remote.close()
        if viewer is not None:
            viewer.close()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
