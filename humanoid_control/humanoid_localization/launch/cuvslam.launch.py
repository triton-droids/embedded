"""cuVSLAM on /stereo/{left,right}; run inside the Isaac ROS 3.2 container (scripts/run_cuvslam.sh)."""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'cuvslam.yaml')
    slam = ComposableNode(
        package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node', parameters=[params],
        remappings=[('visual_slam/image_0', '/stereo/left/image_raw'),
                    ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                    ('visual_slam/image_1', '/stereo/right/image_raw'),
                    ('visual_slam/camera_info_1', '/stereo/right/camera_info')])
    return LaunchDescription([ComposableNodeContainer(
        name='visual_slam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[slam], output='screen')])
