from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Arguments
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
    
    # Python CAN node
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
    
    # C++ Control node
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
    
    return LaunchDescription([
        motor_config_arg,
        control_rate_arg,
        enable_rl_arg,
        rl_model_path_arg,
        python_can_node,
        cpp_control_node,
        LogInfo(msg=[
            'Hybrid control system launched:\n',
            '  - Python CAN node: handles CAN communication\n',
            '  - C++ Control node: handles control and RL inference\n',
            '  - Control rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            '  - Note: Legacy debug node (motor_control_node_debug) can run separately\n',
            '    to provide /robstride_joint_control service for configuration'
        ]),
    ])
