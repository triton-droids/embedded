"""Launch the camera, detector, single-axis tracker, and serial micro-ROS Agent.

Each component can be disabled when it is managed elsewhere. The default Agent
runs as a foreground Docker child so launch owns its lifetime; ``native`` uses
an installed ROS package instead. All enabled components share ROS_DOMAIN_ID.
No fixed startup delay is required: the MCU probes for its Agent, and the tracker
waits for calibrated, fresh detections before publishing any angle commands.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    """Resolve arguments and return only the requested process actions.

    Args:
        context: LaunchContext containing resolved CLI/default configurations.

    Returns:
        A list of Node/ExecuteProcess actions managed by this launch service.

    Raises:
        ValueError: A boolean, numeric argument, or Agent mode is malformed.

    Package lookup for a disabled component is avoided, allowing tracker-only
    development without a local camera driver or native Agent installation.
    """
    def value(name):
        """Return a launch configuration as text before explicit type conversion."""
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        """Parse strict true/false values rather than treating nonempty text as true."""
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    def stop_with(name):
        """Build an exit handler that shuts down all processes owned by this launch."""
        # Loss of any managed process stops the whole launch, including tracker.
        return [EmitEvent(event=Shutdown(reason=f'{name} exited'))]

    actions = []
    if boolean('start_agent'):
        mode = value('agent_mode')
        port = value('serial_port')
        baud = value('baud')
        if mode == 'docker':
            # Device mapping also accepts a stable host /dev/serial/by-id path.
            # Host IPC lets Fast DDS share memory with host-side ROS processes.
            # Pass an argv list: no shell quoting or interactive sudo is involved.
            actions.append(ExecuteProcess(
                cmd=['docker', 'run', '--rm', '--init',
                     '--name', 'head-tracking-agent', '--network', 'host',
                     '--ipc', 'host', '--device', f'{port}:/dev/head_mcu',
                     '--env', f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}',
                     value('agent_image'), 'serial', '--dev', '/dev/head_mcu',
                     '-b', baud, '-v4'],
                output='screen', on_exit=stop_with('micro-ROS Agent')))
        elif mode == 'native':
            actions.append(Node(
                package='micro_ros_agent', executable='micro_ros_agent',
                arguments=['serial', '--dev', port, '-b', baud, '-v4'],
                output='screen', on_exit=stop_with('micro-ROS Agent')))
        else:
            raise ValueError('agent_mode must be docker or native')

    if boolean('start_camera'):
        # Pixel tracking requires RGB and intrinsics only. Optional aligned depth
        # enables the detector's existing 3D path without creating a point cloud.
        depth = boolean('enable_depth')
        actions.append(Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            namespace='camera', name='camera', output='screen',
            parameters=[{
                'enable_color': True, 'enable_depth': depth,
                'align_depth.enable': depth,
                'enable_infra1': False, 'enable_infra2': False,
                'enable_gyro': False, 'enable_accel': False,
                'pointcloud.enable': False,
                'rgb_camera.color_profile': value('color_profile'),
            }], on_exit=stop_with('camera')))

    if boolean('start_detector'):
        # Leave backend/model defaults to the compiled detector unless overridden.
        parameters = {
            'color_topic': value('color_topic'),
            'camera_info_topic': value('camera_info_topic'),
            'depth_topic': value('depth_topic'),
            'show_image': boolean('show_image'),
        }
        if value('model_path'):
            parameters['model_path'] = value('model_path')
        if value('use_cuda') != 'auto':
            parameters['use_cuda'] = boolean('use_cuda')
        actions.append(Node(
            package='humanoid_vision', executable='ball_detection_node',
            parameters=[parameters], output='screen',
            remappings=[('/vision/target_center', value('target_topic'))],
            on_exit=stop_with('detector')))

    if boolean('start_tracker'):
        # launch configurations are strings; ROS double parameters require floats.
        parameters = {name: float(value(name)) for name in (
            'direction', 'kp', 'deadband_px', 'min_angle_rad', 'max_angle_rad',
            'initial_angle_rad', 'max_speed_rad_s', 'max_accel_rad_s2',
            'target_timeout_s', 'control_rate_hz')}
        parameters.update({name: value(name) for name in (
            'axis', 'camera_info_topic', 'target_topic', 'command_topic')})
        actions.append(Node(
            package='humanoid_vision', executable='track_node',
            parameters=[parameters], output='screen',
            on_exit=stop_with('tracker')))
    return actions


def generate_launch_description():
    """Declare user-facing defaults and defer process construction to setup().

    Component switches are independent. A tracker-only build must disable the
    detector; external camera/Agent processes are never stopped by this launch.
    """
    arguments = [
        ('start_camera', 'true', 'Start the RealSense color camera'),
        ('start_agent', 'true', 'Start Agent; false when one already owns serial'),
        ('start_detector', 'true', 'Start box detection'),
        ('start_tracker', 'true', 'Start angle controller'),
        ('agent_mode', 'docker', 'docker or native (installed ROS Agent package)'),
        ('agent_image', 'microros/micro-ros-agent:jazzy', 'Docker Agent image'),
        ('serial_port', '/dev/ttyACM0', 'ESP32 device; /dev/serial/by-id path also works'),
        ('baud', '460800', 'Must match MCU firmware'),
        ('enable_depth', 'false', 'Enable aligned depth only if 3D is needed'),
        ('color_profile', '640,480,30', 'RealSense width,height,fps'),
        ('color_topic', '/camera/camera/color/image_raw', 'Detection color input'),
        ('camera_info_topic', '/camera/camera/color/camera_info', 'Color intrinsics'),
        ('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw', 'Aligned depth'),
        ('target_topic', '/vision/target_center', 'Pixel center connecting detector/tracker'),
        ('command_topic', '/head/servo_target_rad', 'MCU target radians'),
        ('show_image', 'false', 'Display box and image centers (requires GUI)'),
        ('use_cuda', 'auto', 'auto uses detector build default; or true/false'),
        ('model_path', '', 'Optional override of detector model'),
        ('axis', 'yaw', 'yaw or pitch; one servo only'),
        ('direction', '-1.0', 'Reverse to 1.0 if error grows'),
        ('kp', '1.0', 'Visual P gain'),
        ('deadband_px', '8.0', 'Image-center deadband'),
        ('min_angle_rad', '-0.6', 'Lower command limit'),
        ('max_angle_rad', '0.6', 'Upper command limit'),
        ('initial_angle_rad', '0.0', 'Initial joint command estimate'),
        ('max_speed_rad_s', '0.3', 'Maximum command velocity'),
        ('max_accel_rad_s2', '0.8', 'Maximum command acceleration'),
        ('target_timeout_s', '0.15', 'Detection freshness limit'),
        ('control_rate_hz', '50.0', 'MCU command refresh rate'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ] + [OpaqueFunction(function=setup)])
