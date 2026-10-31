from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'use_viewer': 'true', 'auto_start': 'false', 'hold_first_frame': 'false',
        'log_level': 'warn', 'ros_domain_id': '0', 'web_port': '8080',
        'wbc_python_executable': 'python3', 'wbc_tracking_mode': 'policy_impedance',
        'wbc_reference_end_behavior': 'loop', 'wbc_command_rate_hz': '200.0',
        'wbc_reference_loop_start_frame': '50', 'wbc_reference_loop_end_frame': '251',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('humanoid_sim'), 'launch', 'sim.launch.py'])),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in defaults},
                'enable_wbc': 'true', 'enable_policy': 'true',
                'connect_policy': 'true', 'minimal': 'false', 'enable_vision': 'false',
            }.items(),
        ),
    ])
