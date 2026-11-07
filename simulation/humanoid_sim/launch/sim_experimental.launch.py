"""sim.launch.py with the experimental timing fixes of 2026-11-07.

Every sim.launch.py argument is accepted and passed through unchanged. On top
of it this launch sets:

  rmw_implementation (rmw_cyclonedds_cpp)
      DDS for every node of the launch. With Fast DDS 2.14 (the ROS default) a
      publish() can block for ~330 ms about 2 s after another node joins the
      graph, most often one with service clients of the simulator such as
      cpp_control_node, sim_autostart or a `ros2 service call`. rclpy holds
      the GIL in that call, so the whole simulator stalls and the safety IMU
      freshness check (250 ms) or the policy input timeout (100 ms) latches.
      This did not reproduce with CycloneDDS.
  cyclonedds_config (config/cyclonedds_sim.xml)
      Socket receive buffer for camera images; used only with CycloneDDS and
      only when CYCLONEDDS_URI is not already set in the environment.
  gil_switch_interval_s (0.0005)
      mujoco_sim's Python GIL switch interval. With the default 5 ms the
      physics thread can wait a full 5 ms step for the executor or camera
      thread; overruns fell from 82-339 to 8-13 per 30 s.

Measured on a native Ubuntu 24.04 i7-1165G7 host (see the humanoid_sim README).
One-click start: scripts/sim_experimental.sh [arguments]
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def dds_environment(context):
    rmw = LaunchConfiguration('rmw_implementation').perform(context)
    config = LaunchConfiguration('cyclonedds_config').perform(context)
    actions = [SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw)]
    if rmw == 'rmw_cyclonedds_cpp' and config:
        if os.environ.get('CYCLONEDDS_URI'):
            actions.append(LogInfo(msg='CYCLONEDDS_URI is already set; not applying '
                                       f'{config}'))
        else:
            actions.append(SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + config))
    actions.append(LogInfo(msg=f'Experimental simulation: RMW_IMPLEMENTATION={rmw}, '
                               'gil_switch_interval_s='
                               + LaunchConfiguration('gil_switch_interval_s').perform(context)))
    return actions


def generate_launch_description():
    share = FindPackageShare('humanoid_sim')
    return LaunchDescription([
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_cyclonedds_cpp',
                              description='DDS for every node of the launch'),
        DeclareLaunchArgument('cyclonedds_config',
                              default_value=PathJoinSubstitution([share, 'config',
                                                                  'cyclonedds_sim.xml']),
                              description='CycloneDDS XML; empty skips it'),
        DeclareLaunchArgument('gil_switch_interval_s', default_value='0.0005',
                              description='mujoco_sim Python GIL switch interval'),
        OpaqueFunction(function=dds_environment),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sim.launch.py'])),
            launch_arguments={
                'gil_switch_interval_s': LaunchConfiguration('gil_switch_interval_s'),
            }.items()),
    ])
