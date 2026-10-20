"""Workspace entry point for the installed humanoid_vision launch.

Build the workspace and source install/setup.bash before using this entry point.
Launch arguments are declared by the included package launch.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('humanoid_vision'),
                'launch',
                'head_tracking.launch.py',
            ]))
        ),
    ])
