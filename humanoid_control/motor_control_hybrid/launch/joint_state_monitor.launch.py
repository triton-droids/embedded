"""Publish selected feedback or target angles for a remote RViz computer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_topic', default_value='/policy/joint_states',
                              description='Feedback topic; use /policy/target_angles for commanded pose'),
        DeclareLaunchArgument('output_topic', default_value='/joint_states'),
        Node(package='motor_control_hybrid', executable='joint_state_monitor_node',
             output='screen', parameters=[{
                 'source_topic': LaunchConfiguration('source_topic'),
                 'output_topic': LaunchConfiguration('output_topic'),
             }]),
    ])
