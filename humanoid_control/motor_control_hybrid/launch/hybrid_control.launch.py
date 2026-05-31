from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        description='CAN feedback polling rate for idle motors'
    )

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout_s',
        default_value='0.5',
        description='Command timeout in seconds. 0.0 holds the last command indefinitely.'
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
        default_value='true',
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

    # -------------------- Fake motor node --------------------
    fake_motor_node = Node(
        package='motor_control_hybrid',
        executable='fake_motor_node',
        name='fake_motor_node',
        condition=IfCondition(LaunchConfiguration('enable_fake_motor')),
        output='screen',
        parameters=[
            {'joint_names': ['test_joint', 'test_joint2']},
            {'publish_rate_hz': LaunchConfiguration('control_rate_hz')},
        ],
    )

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
        ],
    )

    # -------------------- C++ Control node --------------------
    cpp_control_node = Node(
        package='motor_control_hybrid',
        executable='cpp_control_node',
        name='cpp_control_node',
        condition=IfCondition(LaunchConfiguration('enable_cpp_control')),
        output='screen',
        parameters=[
            {'control_rate_hz': LaunchConfiguration('control_rate_hz')},
            {'cmd_timeout_s': LaunchConfiguration('cmd_timeout_s')},
            {'enable_rl': LaunchConfiguration('enable_rl')},
            {'rl_model_path': LaunchConfiguration('rl_model_path')},
        ],
    )

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
        motor_config_arg,
        control_rate_arg,
        feedback_poll_rate_arg,
        cmd_timeout_arg,
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
        fake_motor_node,
        real_python_can_node,
        cpp_control_node,
        sdk_gateway_node,
        websocket_ui_node,
        policy_bridge_node,
        LogInfo(msg=[
            'Hybrid control system launched:\n',
            '  - Python CAN node: handles CAN communication\n',
            '  - C++ Control node: handles control and RL inference\n',
            '  - SDK gRPC gateway: ', LaunchConfiguration('sdk_grpc_addr'), '\n',
            '  - Policy bridge enabled: ', LaunchConfiguration('enable_policy_bridge'), '\n',
            '  - Double-pendulum UI: http://', LaunchConfiguration('websocket_host'), ':', LaunchConfiguration('websocket_port'), '\n',
            '  - Control rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            '  - Legacy debug node can still run separately if needed\n',
        ]),
    ])
