from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
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

    enable_gateway_arg = DeclareLaunchArgument(
        'enable_gateway',
        default_value='true',
        description='Start HTTP gateway node'
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

    gateway_repeat_publish_hz_arg = DeclareLaunchArgument(
        'gateway_repeat_publish_hz',
        default_value='50.0',
        description='Gateway repeat rate for the last published desired command. 0 disables repeat.'
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

    # -------------------- HTTP Gateway node --------------------
    gateway_node = Node(
        package='motor_control_hybrid',
        executable='target_gateway_node',
        name='target_gateway_node',
        condition=IfCondition(LaunchConfiguration('enable_gateway')),
        output='screen',
        parameters=[
            {'host': LaunchConfiguration('gateway_host')},
            {'port': LaunchConfiguration('gateway_port')},
            {'topic_out': 'motor_commands'},
            {'default_kp': LaunchConfiguration('gateway_default_kp')},
            {'default_kd': LaunchConfiguration('gateway_default_kd')},
            {'default_mode': LaunchConfiguration('gateway_default_mode')},
            {'repeat_publish_hz': LaunchConfiguration('gateway_repeat_publish_hz')},
            {'repeat_topics': ['/desired_motor_subset']},
        ],
    )

    return LaunchDescription([
        motor_config_arg,
        control_rate_arg,
        feedback_poll_rate_arg,
        cmd_timeout_arg,
        enable_rl_arg,
        enable_cpp_control_arg,
        enable_gateway_arg,
        rl_model_path_arg,
        gateway_host_arg,
        gateway_port_arg,
        gateway_default_kp_arg,
        gateway_default_kd_arg,
        gateway_default_mode_arg,
        gateway_repeat_publish_hz_arg,
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
