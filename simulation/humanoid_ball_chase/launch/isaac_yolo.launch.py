"""GPU YOLO pipeline with Isaac ROS 3.2 (runs in the Isaac container: scripts/run_isaac_yolo.sh).

  color image -> DnnImageEncoder (resize with padding, /255) -> TensorRTNode -> YoloV8DecoderNode
  -> /isaac_yolo/detections (vision_msgs/Detection2DArray, boxes in network pixels)

Every stage is a composable node in one container, so tensors stay on the GPU
(NITROS). The models in assets/ are Ultralytics YOLO11n detect exports whose
output [1, 4 + classes, 8400] has the YOLOv8 layout the decoder expects.
td-perception classes: 0 ball, 1 goalpost, 2 robot, 3 L-, 4 T-, 5 X-intersection,
6 penalty mark.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ASSETS = '/workspaces/humanoid/simulation/humanoid_ball_chase/assets'


def generate_launch_description():
    arg = LaunchConfiguration
    args = [
        DeclareLaunchArgument('engine_file_path', default_value=f'{ASSETS}/td-perception-fp16.engine'),
        DeclareLaunchArgument('model_file_path', default_value=f'{ASSETS}/td-perception.onnx'),
        DeclareLaunchArgument('num_classes', default_value='7'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.30'),
        DeclareLaunchArgument('nms_threshold', default_value='0.45'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('input_image_width', default_value='424'),
        DeclareLaunchArgument('input_image_height', default_value='240'),
    ]
    container = ComposableNodeContainer(
        name='isaac_yolo_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[
            ComposableNode(
                name='tensor_rt', package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'engine_file_path': arg('engine_file_path'),
                    'model_file_path': arg('model_file_path'),
                    'input_tensor_names': ['input_tensor'], 'input_binding_names': ['images'],
                    'output_tensor_names': ['output_tensor'], 'output_binding_names': ['output0'],
                    'force_engine_update': False,
                }],
                # Isaac naming: TensorRTNode reads 'tensor_pub' and writes 'tensor_sub'.
                remappings=[('tensor_pub', '/isaac_yolo/input_tensor'), ('tensor_sub', '/isaac_yolo/tensor')]),
            ComposableNode(
                name='yolov8_decoder', package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                parameters=[{
                    'tensor_name': 'output_tensor',
                    'num_classes': arg('num_classes'),
                    'confidence_threshold': arg('confidence_threshold'),
                    'nms_threshold': arg('nms_threshold'),
                }],
                remappings=[('tensor_sub', '/isaac_yolo/tensor'), ('detections_output', '/isaac_yolo/detections')]),
        ])
    encoder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_dnn_image_encoder'), 'launch', 'dnn_image_encoder.launch.py')),
        launch_arguments={
            'input_image_width': arg('input_image_width'),
            'input_image_height': arg('input_image_height'),
            'network_image_width': '640', 'network_image_height': '640',
            'image_mean': '[0.0, 0.0, 0.0]', 'image_stddev': '[1.0, 1.0, 1.0]',
            'enable_padding': 'True',
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'isaac_yolo_container',
            'dnn_image_encoder_namespace': 'isaac_yolo_encoder',
            'image_input_topic': arg('image_topic'),
            'camera_info_input_topic': arg('camera_info_topic'),
            'tensor_output_topic': '/isaac_yolo/input_tensor',
        }.items())
    return LaunchDescription(args + [container, encoder])
