from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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

    enable_rl_arg = DeclareLaunchArgument(
        'enable_rl',
        default_value='false',
        description='Enable RL policy inference'
    )

    rl_model_path_arg = DeclareLaunchArgument(
        'rl_model_path',
        default_value='',
        description='Path to RL model (ONNX) file'
    )

    gateway_host_arg = DeclareLaunchArgument(
        'gateway_host',
        default_value='127.0.0.1',
        description='HTTP gateway host'
    )

    gateway_port_arg = DeclareLaunchArgument(
        'gateway_port',
        default_value='8080',
        description='HTTP gateway port'
    )

    gateway_default_kp_arg = DeclareLaunchArgument(
        'gateway_default_kp',
        default_value='10.0',
        description='Default KP used by gateway when request does not provide kp'
    )

    gateway_default_kd_arg = DeclareLaunchArgument(
        'gateway_default_kd',
        default_value='0.2',
        description='Default KD used by gateway when request does not provide kd'
    )

    gateway_default_mode_arg = DeclareLaunchArgument(
        'gateway_default_mode',
        default_value='motion',
        description='Default mode for gateway: velocity/position/motion/enable/disable'
    )

    # -------------------- Python CAN node --------------------
    python_can_node = Node(
        package='motor_control_hybrid',
        executable='python_can_node',
        name='python_can_node',
        output='screen',
        parameters=[
            {'motor_config_file': LaunchConfiguration('motor_config_file')},
            {'publish_rate_hz': LaunchConfiguration('control_rate_hz')},
        ],
    )

    # -------------------- C++ Control node --------------------
    cpp_control_node = Node(
        package='motor_control_hybrid',
        executable='cpp_control_node',
        name='cpp_control_node',
        output='screen',
        parameters=[
            {'control_rate_hz': LaunchConfiguration('control_rate_hz')},
            {'enable_rl': LaunchConfiguration('enable_rl')},
            {'rl_model_path': LaunchConfiguration('rl_model_path')},
        ],
    )

    # -------------------- HTTP Gateway node --------------------
    gateway_node = Node(
        package='motor_control_hybrid',
        executable='target_gateway_node',
        name='target_gateway_node',
        output='screen',
        parameters=[
            {'host': LaunchConfiguration('gateway_host')},
            {'port': LaunchConfiguration('gateway_port')},
            {'topic_out': 'motor_commands'},
            {'default_kp': LaunchConfiguration('gateway_default_kp')},
            {'default_kd': LaunchConfiguration('gateway_default_kd')},
            {'default_mode': LaunchConfiguration('gateway_default_mode')},
        ],
    )

    return LaunchDescription([
        motor_config_arg,
        control_rate_arg,
        enable_rl_arg,
        rl_model_path_arg,
        gateway_host_arg,
        gateway_port_arg,
        gateway_default_kp_arg,
        gateway_default_kd_arg,
        gateway_default_mode_arg,
        python_can_node,
        cpp_control_node,
        gateway_node,
        LogInfo(msg=[
            'Hybrid control system launched:\n',
            '  - Python CAN node: handles CAN communication\n',
            '  - C++ Control node: handles control and RL inference\n',
            '  - HTTP Gateway node: accepts POST /target and publishes to motor_commands\n',
            '  - Control rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            '  - Gateway: http://', LaunchConfiguration('gateway_host'), ':', LaunchConfiguration('gateway_port'), '\n',
            '  - Legacy debug node can still run separately if needed\n',
        ]),
    ])
