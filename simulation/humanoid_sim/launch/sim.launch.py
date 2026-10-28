"""Run the robot software stack against the MuJoCo simulation.

Hardware-facing processes are replaced by the ``mujoco_sim`` process:

  python_can_node / encos_can_node  -> sim_motor_backend
  body IMU MCU + micro-ROS Agent    -> sim_body_imu
  head MCU + micro-ROS Agent        -> sim_head_mcu
  realsense2_camera_node            -> sim_camera (only with enable_vision)

The default minimal loop runs tracking_policy_node and the safety supervisor.
minimal:=false also runs cpp_control_node, Web monitoring, the joint-state relay
and optional vision. With connect_policy:=true the minimal loop sends the
latest policy output directly to the simulated backend; the full loop routes
through C++ (the hardware launches keep the policy isolated). By default no launch action enables motors; see the package
README. auto_start:=true adds sim_autostart, which enables the motors and
releases the gantry only after fresh motion commands reach the backend.

The stack starts only after the simulator emits its readiness marker. Otherwise
the safety supervisor's 3 s IMU startup grace can expire while MuJoCo loads,
exactly as it would with a late IMU MCU on hardware.

DDS discovery is limited to this host by default so simulated feedback cannot
reach a robot on the same network and domain.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

# Must match humanoid_sim.main.READY_MESSAGE.
SIM_READY = b'Simulation ready'


def include(package, filename, arguments, condition=None):
    return GroupAction(scoped=True, condition=condition, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=arguments.items())])


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def simulation_and_stack(context):
    """Start the simulator, then the robot stack once the simulator is ready."""
    arg = LaunchConfiguration
    registry_path = arg('motor_config_file').perform(context)
    with open(registry_path) as stream:
        registry = yaml.safe_load(stream)
    params = registry.get('motor_control_node', {}).get('ros__parameters', registry)
    motors = params['motors']
    connect_policy = flag(context, 'enable_policy') and flag(context, 'connect_policy')
    minimal = flag(context, 'minimal')
    if minimal and flag(context, 'enable_vision'):
        raise ValueError('enable_vision requires minimal:=false')

    # No name=: a __node remap would rename every node in this multi-node process.
    simulator = Node(
            package='humanoid_sim', executable='mujoco_sim',
            # stdout carries the readiness handshake; diagnostics stay on stderr.
            output={'stdout': 'log', 'stderr': 'screen'},
            ros_arguments=['--log-level', arg('log_level')],
            parameters=[arg('sim_config_file'), {
                'motor_config_file': registry_path,
                'use_viewer': flag(context, 'use_viewer'),
                'enable_camera': flag(context, 'enable_vision'),
                'start_base_held': flag(context, 'start_base_held'),
                'target_image': arg('target_image').perform(context),
                'command_timeout_s': float(arg('can_command_timeout_s').perform(context)),
                'policy_commands_topic': '/policy/desired_motor_subset' if minimal and connect_policy else '',
                **({'telemetry_rate_hz': 0.0, 'enable_head': False,
                    'enable_target': False, 'ground_truth_rate_hz': 10.0} if minimal else {}),
            }, {'enable_depth': flag(context, 'enable_depth')}],
        # Without the simulator every other node would only report stale inputs.
        on_exit=[EmitEvent(event=Shutdown(reason='mujoco_sim exited'))],
    )
    stack = []
    if not minimal:
        stack += [
            # Same configuration as hybrid_control.launch.py's make_cpp_control, except
            # that it talks to the real backend service names (served by the simulator).
            Node(
                package='motor_control_hybrid', executable='cpp_control_node',
                name='cpp_control_node', output='screen',
                ros_arguments=['--log-level', arg('log_level')],
                parameters=[{
                    'joint_names': list(motors),
                    'simulation_zero_enabled': False,
                    'home_joint_lower': [float(cfg.get('min_position', float('nan')))
                                         for cfg in motors.values()],
                    'home_joint_upper': [float(cfg.get('max_position', float('nan')))
                                         for cfg in motors.values()],
                    'control_rate_hz': float(arg('control_rate_hz').perform(context)),
                    'cmd_timeout_s': float(arg('cmd_timeout_s').perform(context)),
                    'control_rt_enabled': False,
                }],
                remappings=[('/desired_motor_subset', '/policy/desired_motor_subset')]
                if connect_policy else [],
            ),
        ]
    if flag(context, 'enable_policy'):
        contract = params['model_contract']
        stack.append(Node(
            package='motor_control_hybrid', executable='tracking_policy_node', output='screen',
            ros_arguments=['--log-level', arg('log_level')],
            parameters=[{
                'model_path': arg('model_path'),
                'control_rate_hz': float(contract['control_hz']),
                'motor_config_file': registry_path,
                'imu_topic': '/imu/data_raw',
                'orientation_source': arg('orientation_source'),
                'hold_first_frame': ParameterValue(arg('hold_first_frame'), value_type=bool),
                'joint_states_topic': '/joint_states',
                'joint_feedback_mode': 'joint_states',
                'calibration_seconds': ParameterValue(arg('calibration_seconds'), value_type=float),
            }]))
    if connect_policy and flag(context, 'auto_start'):
        stack.append(Node(
            package='humanoid_sim', executable='sim_autostart', output='screen',
            ros_arguments=['--log-level', arg('log_level')],
            parameters=[{'release_delay_s': ParameterValue(arg('release_delay_s'),
                                                           value_type=float)}]))
    stack += [
        # Supervisor + Web monitor + measured-feedback relay/RViz, as in control_test.
        include('humanoid_safety', 'safety.launch.py', {
            'mode': 'supervisor' if minimal else 'all',
            'log_level': arg('log_level'),
            'motor_config_file': arg('motor_config_file'),
            'safety_config': arg('safety_config_file'),
            'fall_detection_enabled': arg('fall_detection_enabled'),
            'trip_on_imu_timeout': arg('trip_on_imu_timeout'),
            'ros_domain_id': arg('ros_domain_id'),
            'source_topic': '/joint_states',
            'port': arg('web_port'),
            'start_display': arg('use_rviz'),
            'use_rviz': arg('use_rviz'),
        }),
        include('humanoid_vision', 'head_tracking.launch.py', {
            'log_level': arg('log_level'),
            'start_camera': 'false',
            'start_agent': 'false',
            'start_detector': arg('start_detector'),
            'start_tracker': 'true',
            'use_cuda': arg('use_cuda'),
        }, condition=IfCondition(arg('enable_vision'))),
    ]

    started = []

    def start_stack_once(event):
        if started or SIM_READY not in event.text:
            return None
        started.append(True)
        return stack

    return [simulator, RegisterEventHandler(OnProcessIO(
        target_action=simulator, on_stdout=start_stack_once, on_stderr=start_stack_once))]


def generate_launch_description():
    arg = LaunchConfiguration
    share = FindPackageShare
    defaults = [
        ('minimal', 'true', 'Direct simulated policy loop; false restores C++ forwarding and monitors'),
        ('log_level', 'warn', 'ROS log severity: warn retains warnings, errors and fatal diagnostics'),
        ('use_viewer', 'true', 'Open the interactive MuJoCo viewer'),
        ('use_rviz', 'false', 'Start robot_state_publisher display and RViz'),
        ('enable_policy', 'true', 'Run tracking_policy_node on simulated IMU/joint feedback'),
        ('orientation_source', 'message', 'Policy gravity source: message quaternion or legacy complementary'),
        ('hold_first_frame', 'false', 'Keep reference frame 0 while the policy continues closed-loop corrections'),
        ('connect_policy', 'true', 'Route /policy/desired_motor_subset into cpp_control_node'),
        ('auto_start', 'false',
         'Enable motors and release the gantry when the policy starts its reference motion'),
        ('release_delay_s', '0.1', 'With auto_start: delay from enable to gantry release'),
        ('enable_vision', 'false', 'Render the head camera and run detector + head tracker'),
        ('start_detector', 'true', 'With enable_vision: false for TRACK_ONLY builds'),
        ('use_cuda', 'false', 'Detector CUDA backend'),
        ('enable_depth', 'false', 'Render aligned depth for the detector 3D output'),
        ('start_base_held', 'true', 'Start on the virtual gantry'),
        ('target_image', '', 'PNG face image on the mocap target board'),
        ('control_rate_hz', '50.0', 'C++ scheduler rate'),
        ('cmd_timeout_s', '0.5', 'C++ scheduler command timeout'),
        ('can_command_timeout_s', '0.5', 'Simulated backend command lifetime'),
        ('calibration_seconds', '2.0', 'Policy gyro calibration; keep the base held meanwhile'),
        ('fall_detection_enabled', '', 'Override safety.yaml fall detection'),
        ('trip_on_imu_timeout', '', 'Override safety.yaml IMU timeout trip'),
        ('localhost_only', '1', 'Confine DDS discovery to this host (1) or not (0)'),
        ('ros_domain_id', EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'), 'ROS domain'),
        ('web_port', '8080', 'Read-only motor Web monitor port'),
    ]
    paths = [
        ('motor_config_file', [share('motor_control_hybrid'), 'config', 'motors.yaml']),
        ('safety_config_file', [share('humanoid_safety'), 'config', 'safety.yaml']),
        ('sim_config_file', [share('humanoid_sim'), 'config', 'sim.yaml']),
        ('model_path', [share('motor_control_hybrid'), 'models', '20260914_170827',
                        '20260914_170827.onnx']),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value, description=text)
         for name, value, text in defaults]
        + [DeclareLaunchArgument(name, default_value=PathJoinSubstitution(parts))
           for name, parts in paths]
        + [
            SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', arg('localhost_only')),
            OpaqueFunction(function=simulation_and_stack),
        ])
