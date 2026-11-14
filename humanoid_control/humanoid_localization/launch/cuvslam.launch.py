"""cuVSLAM on /stereo/{left,right} (+ /head/imu); run inside the Isaac ROS 4.6 container (scripts/run_cuvslam.sh).

imu_fusion:=true (default) fuses the camera-carrier IMU (/head/imu, frame head_imu_link);
imu_fusion:=false is pure stereo VO, as before.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

PARAMS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'cuvslam.yaml')
REMAPPINGS = [('visual_slam/image_0', '/stereo/left/image_raw'),
              ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
              ('visual_slam/image_1', '/stereo/right/image_raw'),
              ('visual_slam/camera_info_1', '/stereo/right/camera_info'),
              ('visual_slam/imu', '/head/imu')]
IMU_FUSION_ARG = DeclareLaunchArgument(
    'imu_fusion', default_value='true',
    description='Fuse the camera-carrier IMU /head/imu (false: pure stereo VO)')
SIM_TIME_ARG = DeclareLaunchArgument(
    'use_sim_time', default_value='true',
    description='Follow /clock (ball_chase sim_time:=true, the default); false for wall-clock runs')


def slam_node(params=PARAMS):
    """The VisualSlamNode description (isaac_yolo.launch.py mirrors it for with_cuvslam)."""
    return ComposableNode(
        package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[params, {
            'tracking_mode': ParameterValue(PythonExpression(["1 if '", LaunchConfiguration('imu_fusion'), "'.lower() == 'true' else 0"]), value_type=int),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        remappings=REMAPPINGS)


def generate_launch_description():
    return LaunchDescription([IMU_FUSION_ARG, SIM_TIME_ARG, ComposableNodeContainer(
        name='visual_slam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[slam_node()],
        output='screen')])
