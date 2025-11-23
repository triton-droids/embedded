from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('motor_control_node'))

    yaml_file = pkg_share / 'config' / 'motor_can_map.yaml'

    return LaunchDescription([
        Node(
            package='motor_control_node',
            executable='motor_control',   
            name='motor_control_node',
            output='screen',
            parameters=[str(yaml_file)],
        )
    ])
