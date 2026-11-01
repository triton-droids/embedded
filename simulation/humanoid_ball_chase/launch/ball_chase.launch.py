"""Camera/ball pursuit integration. Bundled policy does not consume velocity commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Run bundled reference motion; does not enable velocity pursuit'),
        DeclareLaunchArgument('ros_domain_id', default_value='61'),
        DeclareLaunchArgument('web_port', default_value='8081'),
        DeclareLaunchArgument('enable_astar', default_value='false',
                              description='Plan paths from external OccupancyGrid and PoseStamped goals'),
        DeclareLaunchArgument('vision_target_timeout_s', default_value='1.0',
                              description='Maximum image/receipt age for CPU football inference; seconds'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('humanoid_sim'), 'launch', 'sim.launch.py'])), launch_arguments={
                'scene_builder': 'humanoid_ball_chase.scene:build_scene',
                'sim_config_file': PathJoinSubstitution([FindPackageShare('humanoid_ball_chase'), 'config', 'scene.yaml']),
                'enable_vision': 'true', 'enable_depth': 'true', 'start_detector': 'false',
                'vision_target_timeout_s': arg('vision_target_timeout_s'),
                'use_viewer': arg('use_viewer'), 'auto_start': arg('auto_start'),
                'ros_domain_id': arg('ros_domain_id'), 'web_port': arg('web_port'),
            }.items()),
        Node(package='humanoid_ball_chase', executable='ball_chase_vision', output='screen',
             parameters=[{'target_timeout_s':ParameterValue(arg('vision_target_timeout_s'), value_type=float)}]),
        Node(package='humanoid_ball_chase', executable='ball_chase_astar',
             condition=IfCondition(arg('enable_astar')), output='screen'),
        Node(package='humanoid_vision', executable='track_node', name='pitch_tracker',
             parameters=[{'axis':'pitch', 'command_topic':'/head/tilt_target_rad',
                          'direction':1.0, 'initial_angle_rad':0.0,
                          'target_timeout_s':ParameterValue(arg('vision_target_timeout_s'), value_type=float),
                          'max_speed_rad_s':0.3,
                          'max_accel_rad_s2':0.6}], output='screen'),
    ])
