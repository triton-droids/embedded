"""cuVSLAM on the D435 infrared pair; run inside the Isaac ROS 4.6 container
(scripts/run_cuvslam.sh), with the camera started on the host (vio_d435.launch.py).

imu_fusion:=false (default) is pure stereo VO. imu_fusion:=true fuses /head/imu and
needs a measured camera_link -> head_imu_link TF and a common camera/IMU clock.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

PARAMS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'cuvslam.yaml')
CAMERA = '/camera/camera'
REMAPPINGS = [('visual_slam/image_0', f'{CAMERA}/infra1/image_rect_raw'),
              ('visual_slam/camera_info_0', f'{CAMERA}/infra1/camera_info'),
              ('visual_slam/image_1', f'{CAMERA}/infra2/image_rect_raw'),
              ('visual_slam/camera_info_1', f'{CAMERA}/infra2/camera_info'),
              ('visual_slam/imu', '/head/imu')]


def generate_launch_description():
    imu_fusion = LaunchConfiguration('imu_fusion')
    slam = ComposableNode(
        package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[PARAMS, {
            'enable_imu_fusion': ParameterValue(imu_fusion, value_type=bool),
            'tracking_mode': ParameterValue(PythonExpression(
                ["1 if '", imu_fusion, "'.lower() == 'true' else 0"]), value_type=int)}],
        remappings=REMAPPINGS)
    return LaunchDescription([
        DeclareLaunchArgument('imu_fusion', default_value='false',
                              description='Fuse /head/imu (needs extrinsic and clock sync)'),
        ComposableNodeContainer(
            name='visual_slam_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', composable_node_descriptions=[slam],
            output='screen'),
    ])
