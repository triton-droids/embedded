from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def registry_joint_names(context):
    """Read the selected registry's complete order, including custom bench files."""
    with open(LaunchConfiguration('motor_config_file').perform(context)) as stream:
        registry = yaml.safe_load(stream)
    params = registry.get('motor_control_node', {}).get('ros__parameters', registry)
    return list(params['motors'])


def make_cpp_control(context):
    """Configure the control watchdog with the full expected robot."""
    return [Node(
        package='motor_control_hybrid', executable='cpp_control_node',
        name='cpp_control_node', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cpp_control')),
        parameters=[{
            'joint_names': registry_joint_names(context),
            'control_rate_hz': float(LaunchConfiguration('control_rate_hz').perform(context)),
            'cmd_timeout_s': float(LaunchConfiguration('cmd_timeout_s').perform(context)),
            'control_rt_enabled': ParameterValue(LaunchConfiguration('control_rt_enabled'), value_type=bool),
            'control_rt_priority': ParameterValue(LaunchConfiguration('control_rt_priority'), value_type=int),
            'enable_rl': LaunchConfiguration('enable_rl'),
            'rl_model_path': LaunchConfiguration('rl_model_path'),
        }],
    )]


def make_fake_motor(context):
    """Keep fake feedback consistent with the controller's expected joints."""
    return [Node(
        package='motor_control_hybrid', executable='fake_motor_node',
        name='fake_motor_node', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_fake_motor')),
        parameters=[{
            'joint_names': registry_joint_names(context),
            'publish_rate_hz': float(LaunchConfiguration('control_rate_hz').perform(context)),
        }],
    )]


def generate_launch_description():
    # -------------------- Arguments --------------------
    motor_config_arg = DeclareLaunchArgument(
        'motor_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('motor_control_hybrid'),
            'config',
            'motors.yaml'
        ]),
        description='Path to motor configuration YAML file'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='50.0',
        description='Control loop rate in Hz'
    )

    feedback_poll_rate_arg = DeclareLaunchArgument(
        'feedback_poll_hz',
        default_value='50.0',
        description='Legacy compatibility parameter; CAN RX now runs continuously'
    )

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout_s',
        default_value='0.5',
        description='Command timeout in seconds. 0.0 holds the last command indefinitely.'
    )

    can_cmd_timeout_arg = DeclareLaunchArgument(
        'can_command_timeout_s', default_value='0.5',
        description='Positive CAN-local command lifetime; expiry requires explicit re-enable'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Start the latched humanoid safety supervisor'
    )

    safety_config_arg = DeclareLaunchArgument(
        'safety_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_safety'),
            'config',
            'safety.yaml'
        ]),
        description='Path to safety supervisor YAML config'
    )

    enable_rl_arg = DeclareLaunchArgument(
        'enable_rl',
        default_value='false',
        description='Enable RL policy inference'
    )

    enable_cpp_control_arg = DeclareLaunchArgument(
        'enable_cpp_control',
        default_value='true',
        description='Start C++ control node'
    )

    enable_sdk_gateway_arg = DeclareLaunchArgument(
        'enable_sdk_gateway',
        default_value='false',
        description='Start gRPC SDK motor gateway node'
    )

    enable_fake_motor_arg = DeclareLaunchArgument(
        'enable_fake_motor',
        default_value='false',
        description='Start fake motor node instead of real CAN hardware for local testing'
    )

    enable_websocket_ui_arg = DeclareLaunchArgument(
        'enable_websocket_ui',
        default_value='true',
        description='Start browser websocket double-pendulum UI'
    )

    enable_policy_bridge_arg = DeclareLaunchArgument(
        'enable_policy_bridge',
        default_value='false',
        description='Start Torch policy bridge node'
    )

    policy_config_path_arg = DeclareLaunchArgument(
        'policy_config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('motor_control_hybrid'),
            'config',
            'policy_bridge_config.json'
        ]),
        description='Path to policy bridge JSON config'
    )

    rl_model_path_arg = DeclareLaunchArgument(
        'rl_model_path',
        default_value='',
        description='Path to RL model/policy file'
    )

    sdk_grpc_addr_arg = DeclareLaunchArgument(
        'sdk_grpc_addr',
        default_value='127.0.0.1:50052',
        description='gRPC bind address for the SDK motor gateway'
    )

    websocket_host_arg = DeclareLaunchArgument(
        'websocket_host',
        default_value='0.0.0.0',
        description='Double-pendulum websocket UI host'
    )

    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='8765',
        description='Double-pendulum websocket UI port'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='sensor_msgs/Imu topic used by policy bridge'
    )

    # -------------------- Safety supervisor --------------------
    safety_node = Node(
        package='humanoid_safety',
        executable='safety_node',
        name='safety_node',
        condition=IfCondition(LaunchConfiguration('enable_safety')),
        output='screen',
        parameters=[LaunchConfiguration('safety_config_file')],
    )

    # -------------------- Fake motor node --------------------
    fake_motor_node = OpaqueFunction(function=make_fake_motor)

    # -------------------- Python CAN node --------------------
    real_python_can_node = Node(
        package='motor_control_hybrid',
        executable='python_can_node',
        name='python_can_node',
        condition=UnlessCondition(LaunchConfiguration('enable_fake_motor')),
        output='screen',
        parameters=[
            {'motor_config_file': LaunchConfiguration('motor_config_file')},
            {'publish_rate_hz': LaunchConfiguration('control_rate_hz')},
            {'feedback_poll_hz': LaunchConfiguration('feedback_poll_hz')},
            {'feedback_poll_when_idle': True},
            {'command_timeout_s': LaunchConfiguration('can_command_timeout_s')},
            {'can_rt_enabled': ParameterValue(LaunchConfiguration('can_rt_enabled'), value_type=bool)},
            {'can_tx_rt_priority': ParameterValue(LaunchConfiguration('can_tx_rt_priority'), value_type=int)},
            {'can_rx_rt_priority': ParameterValue(LaunchConfiguration('can_rx_rt_priority'), value_type=int)},
        ],
    )

    # -------------------- C++ Control node --------------------
    cpp_control_node = OpaqueFunction(function=make_cpp_control)

    # -------------------- SDK gRPC Gateway node --------------------
    sdk_gateway_node = Node(
        package='motor_control_hybrid',
        executable='motor_sdk_gateway_node',
        name='motor_sdk_gateway_node',
        condition=IfCondition(LaunchConfiguration('enable_sdk_gateway')),
        output='screen',
        parameters=[
            {'grpc_addr': LaunchConfiguration('sdk_grpc_addr')},
        ],
    )

    # -------------------- Browser WebSocket double-pendulum UI --------------------
    websocket_ui_node = Node(
        package='motor_control_hybrid',
        executable='double_pendulum_websocket_node',
        name='double_pendulum_websocket_node',
        condition=IfCondition(LaunchConfiguration('enable_websocket_ui')),
        output='screen',
        parameters=[
            {'host': LaunchConfiguration('websocket_host')},
            {'port': LaunchConfiguration('websocket_port')},
            {'joint_names': ['test_joint', 'test_joint2']},
        ],
    )

    # -------------------- Torch policy bridge --------------------
    policy_bridge_node = Node(
        package='motor_control_hybrid',
        executable='policy_bridge_node',
        name='policy_bridge_node',
        condition=IfCondition(LaunchConfiguration('enable_policy_bridge')),
        output='screen',
        parameters=[
            {'config_path': LaunchConfiguration('policy_config_path')},
            {'policy_path': LaunchConfiguration('rl_model_path')},
            {'imu_topic': LaunchConfiguration('imu_topic')},
            {'control_rate_hz': LaunchConfiguration('control_rate_hz')},
            {'output_topic': '/desired_motor_subset'},
            {'publish_mode': 'motion'},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_rt_enabled', default_value='false'),
        DeclareLaunchArgument('can_tx_rt_priority', default_value='20'),
        DeclareLaunchArgument('can_rx_rt_priority', default_value='21'),
        DeclareLaunchArgument('control_rt_enabled', default_value='false'),
        DeclareLaunchArgument('control_rt_priority', default_value='10'),
        motor_config_arg,
        control_rate_arg,
        feedback_poll_rate_arg,
        cmd_timeout_arg,
        can_cmd_timeout_arg,
        enable_safety_arg,
        safety_config_arg,
        enable_rl_arg,
        enable_cpp_control_arg,
        enable_sdk_gateway_arg,
        enable_fake_motor_arg,
        enable_websocket_ui_arg,
        enable_policy_bridge_arg,
        policy_config_path_arg,
        rl_model_path_arg,
        sdk_grpc_addr_arg,
        websocket_host_arg,
        websocket_port_arg,
        imu_topic_arg,
        safety_node,
        fake_motor_node,
        real_python_can_node,
        cpp_control_node,
        sdk_gateway_node,
        websocket_ui_node,
        policy_bridge_node,
        LogInfo(msg=[
            'Hybrid control system launched:\n',
            '  - Safety supervisor enabled: ', LaunchConfiguration('enable_safety'), '\n',
            '  - Python CAN node: handles CAN communication\n',
            '  - C++ Control node: handles control and RL inference\n',
            '  - SDK gRPC gateway: ', LaunchConfiguration('sdk_grpc_addr'), '\n',
            '  - Policy bridge enabled: ', LaunchConfiguration('enable_policy_bridge'), '\n',
            '  - Double-pendulum UI: http://', LaunchConfiguration('websocket_host'), ':', LaunchConfiguration('websocket_port'), '\n',
            '  - Control rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            '  - Legacy debug node can still run separately if needed\n',
        ]),
    ])
