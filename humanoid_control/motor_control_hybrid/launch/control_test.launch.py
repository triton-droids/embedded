"""Body control + safety + Web + feedback-driven collision RViz, without policy/vision.

Real motors are the default; fake mode is explicit and mutually exclusive with
CAN. No launch action enables, homes or zeros motors. The existing safety-owned
JointState relay preserves measured coordinates and timestamps. The URDF state
publisher performs forward kinematics; world/base translation is not estimated.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=arguments.items())])


def generate_launch_description():
    arg = LaunchConfiguration
    registry = PathJoinSubstitution([FindPackageShare('motor_control_hybrid'), 'config', 'motors.yaml'])
    safety = PathJoinSubstitution([FindPackageShare('humanoid_safety'), 'config', 'safety.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('motor_config_file', default_value=registry),
        DeclareLaunchArgument('safety_config_file', default_value=safety),
        DeclareLaunchArgument('enable_fake_motor', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fall_detection_enabled', default_value='', choices=['', 'true', 'false']),
        DeclareLaunchArgument('trip_on_imu_timeout', default_value='', choices=['', 'true', 'false']),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('cmd_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('can_command_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('display_source_topic', default_value='/joint_states',
                             description='Measured JointState input; policy target display is explicitly selectable'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        include('motor_control_hybrid', 'hybrid_control.launch.py', {
            'motor_config_file': arg('motor_config_file'),
            'enable_fake_motor': arg('enable_fake_motor'),
            'enable_cpp_control': 'true',
            'enable_safety': 'false',  # Exactly one supervisor, supplied below.
            'enable_sdk_gateway': 'false',
            'enable_websocket_ui': 'false',
            'enable_policy_bridge': 'false',
            'enable_rl': 'false',
            'control_rate_hz': arg('control_rate_hz'),
            'cmd_timeout_s': arg('cmd_timeout_s'),
            'can_command_timeout_s': arg('can_command_timeout_s'),
            'can_rt_enabled': 'false',
            'control_rt_enabled': 'false',
        }),
        include('humanoid_safety', 'safety.launch.py', {
            'mode': 'all',
            'motor_config_file': arg('motor_config_file'),
            'safety_config': arg('safety_config_file'),
            'fall_detection_enabled': arg('fall_detection_enabled'),
            'trip_on_imu_timeout': arg('trip_on_imu_timeout'),
            'ros_domain_id': arg('ros_domain_id'),
            # Web always monitors measured feedback, even when RViz displays targets.
            'source_topic': '/joint_states',
            'display_source_topic': arg('display_source_topic'),
            'display_topic': '/safety/visualization/joint_states',
            'host': arg('host'), 'port': arg('port'),
            'start_display': 'false', 'use_rviz': 'false',
        }),
        include('humanoid_safety', 'self_collision_rviz.launch.py', {
            'use_gui': 'false',
            'use_rviz': arg('use_rviz'),
            'joint_states_topic': '/safety/visualization/joint_states',
        }),
    ])
