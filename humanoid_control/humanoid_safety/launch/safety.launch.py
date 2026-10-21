"""Safety supervisor and read-only ROS network monitoring modes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def mode_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    nodes = []
    if mode in ('supervisor', 'all'):
        overrides = {}
        for name in ('fall_detection_enabled', 'trip_on_imu_timeout'):
            if LaunchConfiguration(name).perform(context):
                overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
        nodes.append(Node(package='humanoid_safety', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[LaunchConfiguration('safety_config'), overrides]))
    if mode in ('web', 'monitor', 'all'):
        parameters = {
            'source_topic': LaunchConfiguration('source_topic'),
            'motor_status_topic': LaunchConfiguration('motor_status_topic'),
            'supervisor_status_topic': LaunchConfiguration('supervisor_status_topic'),
            'host': LaunchConfiguration('host'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
        }
        # Empty means auto-detect the optional installed motor registry in the node.
        config = LaunchConfiguration('motor_config_file').perform(context)
        if config:
            parameters['motor_config_file'] = config
        nodes.append(Node(package='humanoid_safety', executable='motor_web_node',
                          output='screen', parameters=[parameters]))
    if mode in ('rviz', 'monitor', 'all'):
        nodes.append(Node(package='humanoid_safety', executable='joint_state_monitor_node',
            output='screen', parameters=[{
                'source_topic': LaunchConfiguration('display_source_topic'),
                'output_topic': LaunchConfiguration('display_topic'),
            }]))
        if LaunchConfiguration('start_display').perform(context).lower() == 'true':
            nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('humanoid_leg_description'),
                                      'launch', 'display.launch.py'])),
                launch_arguments={
                    'use_joint_state_gui': 'false',
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'joint_states_topic': LaunchConfiguration('display_topic'),
                }.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='supervisor',
            choices=['supervisor', 'web', 'rviz', 'monitor', 'all'],
            description='supervisor; read-only web/rviz; monitor=both views; all=supervisor+views'),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        DeclareLaunchArgument('source_topic', default_value='/joint_states'),
        DeclareLaunchArgument('display_source_topic', default_value=LaunchConfiguration('source_topic'),
                             description='JointState source for model display; Web keeps source_topic'),
        DeclareLaunchArgument('motor_status_topic', default_value='/motor_telemetry'),
        DeclareLaunchArgument('supervisor_status_topic', default_value='/safety/status'),
        DeclareLaunchArgument('display_topic', default_value='/safety/visualization/joint_states'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('motor_config_file', default_value=''),
        DeclareLaunchArgument('start_display', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fall_detection_enabled', default_value='', choices=['', 'true', 'false'],
                             description='Override YAML fall detection; empty preserves configuration'),
        DeclareLaunchArgument('trip_on_imu_timeout', default_value='', choices=['', 'true', 'false'],
                             description='Override YAML IMU timeout trip; empty preserves configuration'),
        DeclareLaunchArgument('safety_config', default_value=PathJoinSubstitution(
            [FindPackageShare('humanoid_safety'), 'config', 'safety.yaml'])),
        OpaqueFunction(function=mode_nodes),
    ])
