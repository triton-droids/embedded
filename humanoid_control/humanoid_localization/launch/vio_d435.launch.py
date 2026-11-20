"""Host side of VIO on the robot: D435 infrared pair, camera mount TF, vio_localizer.

cuVSLAM runs in the Isaac ROS container (scripts/run_cuvslam.sh) and publishes
/visual_slam/tracking/odometry, the pose of camera_link. vio_localizer turns VO
steps into base displacements with the TF base_frame -> camera_link and publishes
/localization/odom.

attitude_source:=vo (default) takes the base attitude from VO and assumes the base
is level at start: the control launch runs the body IMU reader with orientation
output off. attitude_source:=imu needs an orientation quaternion on imu_topic.

The camera mount (camera_x .. camera_yaw) has no default: measure it on the robot.
It is published as a static TF, valid only while the head servo holds one pose; the
head has no encoder, so a moving head needs forward kinematics from a measured angle.

start_camera:=false when another launch already owns the D435 (for example
head_tracking.launch.py with enable_infra:=true): one process per camera.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MOUNT = ('camera_x', 'camera_y', 'camera_z', 'camera_roll', 'camera_pitch', 'camera_yaw')


def generate_launch_description():
    arg = LaunchConfiguration
    camera = Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        condition=IfCondition(arg('start_camera')),
        parameters=[{
            'enable_color': False, 'enable_depth': False,
            'enable_infra1': True, 'enable_infra2': True,
            # Passive stereo: the projected pattern moves with the camera and would
            # be tracked as static scenery.
            'depth_module.emitter_enabled': 0,
            'depth_module.infra_profile': arg('infra_profile'),
            'enable_gyro': False, 'enable_accel': False, 'pointcloud.enable': False,
        }])
    mount = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
        arguments=['--x', arg('camera_x'), '--y', arg('camera_y'), '--z', arg('camera_z'),
                   '--roll', arg('camera_roll'), '--pitch', arg('camera_pitch'),
                   '--yaw', arg('camera_yaw'),
                   '--frame-id', arg('base_frame'), '--child-frame-id', 'camera_link'])
    localizer = Node(
        package='humanoid_localization', executable='vio_localizer', output='screen',
        parameters=[{
            'attitude_source': arg('attitude_source'),
            'imu_topic': arg('imu_topic'),
            'base_frame': arg('base_frame'), 'rig_frame': 'camera_link',
            'initial_x': ParameterValue(arg('initial_x'), value_type=float),
            'initial_y': ParameterValue(arg('initial_y'), value_type=float),
            'initial_yaw': ParameterValue(arg('initial_yaw'), value_type=float),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('infra_profile', default_value='640,480,30',
                              description='D435 infrared width,height,fps'),
        DeclareLaunchArgument('attitude_source', default_value='vo', choices=['vo', 'imu'],
                              description='vo: attitude from VO; imu: quaternion on imu_topic'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw',
                              description='Body IMU with an orientation quaternion (imu only)'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        *[DeclareLaunchArgument(name, description='Camera mount on base_frame (m / rad); measure it')
          for name in MOUNT],
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        camera, mount, localizer,
    ])
