"""Safety supervisor and read-only ROS network monitoring modes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


NO_TIMEOUT_S = 1.0e6  # 'Disabled' freshness limits must stay positive and finite.


def node_scoped(context, node, values):
    """A parameter file with ``values`` under ``node:``.

    A launch dictionary becomes a ``/**`` section, and on Humble a node's own
    section in an earlier file (safety.yaml's ``safety_node:``) beats a wildcard
    regardless of order, so dictionary overrides of keys in safety.yaml were
    silently ignored. A node-scoped file given last does override them.
    """
    import tempfile
    import yaml
    resolved = {}
    for key, value in values.items():
        if isinstance(value, ParameterValue):
            value = value.evaluate(context)
        elif hasattr(value, 'perform'):
            value = yaml.safe_load(value.perform(context))
        resolved[key] = value
    with tempfile.NamedTemporaryFile('w', prefix=f'{node}_overrides_', suffix='.yaml',
                                     delete=False) as stream:
        yaml.safe_dump({node: {'ros__parameters': resolved}}, stream)
    return stream.name


def mode_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    nodes = []
    joint_monitor = LaunchConfiguration('start_joint_state_monitor').perform(context).lower() == 'true'
    if mode in ('supervisor', 'all'):
        overrides = {'status_reliability': LaunchConfiguration('status_reliability')}
        for name in ('fall_detection_enabled', 'trip_on_imu_timeout'):
            if LaunchConfiguration(name).perform(context):
                overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
        # Rendering and publishing the DiagnosticArray in the supervisor process
        # let a 317 ms status write delay its IMU callback past the freshness
        # check. A separate process removes that queue; inline restores it.
        external = LaunchConfiguration('start_diagnostics').perform(context).lower() == 'true'
        overrides['status_publisher'] = 'external' if external else 'inline'
        # Simulation-time runs: no freshness trips (IMU, motor feedback, CAN timing
        # are judged on the host's steady clock, which a slowed simulation lags).
        no_timeouts = LaunchConfiguration('disable_timeouts').perform(context).lower() == 'true'
        if no_timeouts:
            overrides.update(trip_on_imu_timeout=False, imu_timeout_s=NO_TIMEOUT_S,
                             motor_feedback_timeout_s=NO_TIMEOUT_S, can_timeout_s=NO_TIMEOUT_S)
        nodes.append(Node(package='humanoid_safety', executable='safety_node',
            name='safety_node', output='screen',
            ros_arguments=['--log-level', LaunchConfiguration('log_level')],
            parameters=[LaunchConfiguration('safety_config'), node_scoped(context, 'safety_node', overrides)]))
        if external:
            nodes.append(Node(package='humanoid_safety',
                executable='safety_diagnostics_node', name='safety_diagnostics',
                output='screen',
                ros_arguments=['--log-level', LaunchConfiguration('log_level')],
                parameters=[{
                    'state_topic': LaunchConfiguration('supervisor_state_topic'),
                    'status_topic': LaunchConfiguration('supervisor_status_topic'),
                    'status_reliability': LaunchConfiguration('status_reliability'),
                    'stale_after_s': NO_TIMEOUT_S if no_timeouts else ParameterValue(
                        LaunchConfiguration('diagnostics_stale_after_s'), value_type=float),
                }]))
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
                          output='screen', ros_arguments=['--log-level', LaunchConfiguration('log_level')],
                          parameters=[parameters]))
    if mode in ('rviz', 'monitor', 'all'):
        if joint_monitor:
            nodes.append(Node(package='humanoid_safety', executable='joint_state_monitor_node',
                output='screen', ros_arguments=['--log-level', LaunchConfiguration('log_level')],
                parameters=[{
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
                    'joint_states_topic': LaunchConfiguration(
                        'display_topic' if joint_monitor else 'display_source_topic'),
                }.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='warn',
            description='ROS diagnostic severity'),
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
        DeclareLaunchArgument('supervisor_state_topic', default_value='/safety/supervisor_state',
            description='Compact snapshot the supervisor publishes for the diagnostics process'),
        DeclareLaunchArgument('start_diagnostics', default_value='true', choices=['true', 'false'],
            description='Render /safety/status in a separate process; false publishes inline'),
        DeclareLaunchArgument('diagnostics_stale_after_s', default_value='3.0',
            description='Publish a latched diagnostic when no snapshot arrives for this long'),
        DeclareLaunchArgument('status_reliability', default_value='reliable',
                             choices=['reliable', 'best_effort']),
        DeclareLaunchArgument('display_topic', default_value='/safety/visualization/joint_states'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('motor_config_file', default_value=''),
        DeclareLaunchArgument('start_joint_state_monitor', default_value='true', choices=['true', 'false'],
                             description='Start the joint-state visualization relay'),
        DeclareLaunchArgument('start_display', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fall_detection_enabled', default_value='', choices=['', 'true', 'false'],
                             description='Override YAML fall detection; empty preserves configuration'),
        DeclareLaunchArgument('disable_timeouts', default_value='false', choices=['true', 'false'],
                              description='Simulation time: no IMU/feedback/CAN freshness trips'),
        DeclareLaunchArgument('trip_on_imu_timeout', default_value='', choices=['', 'true', 'false'],
                             description='Override YAML IMU timeout trip; empty preserves configuration'),
        DeclareLaunchArgument('safety_config', default_value=PathJoinSubstitution(
            [FindPackageShare('humanoid_safety'), 'config', 'safety.yaml'])),
        OpaqueFunction(function=mode_nodes),
    ])
