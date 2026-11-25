"""Start the mode manager (dry run by default) with config/mode_manager.yaml."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true', description='Execute no action'),
        DeclareLaunchArgument('controller', default_value='walk', description='walk or policy'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(package='humanoid_mode_manager', executable='mode_manager', output='screen',
             ros_arguments=['--log-level', arg('log_level')],
             parameters=[PathJoinSubstitution([FindPackageShare('humanoid_mode_manager'), 'config',
                                               'mode_manager.yaml']),
                         {'dry_run': ParameterValue(arg('dry_run'), value_type=bool),
                          'controller': arg('controller'),
                          'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}]),
    ])
