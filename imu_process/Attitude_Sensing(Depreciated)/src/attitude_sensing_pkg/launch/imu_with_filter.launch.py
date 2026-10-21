from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory


def _require_pkg(pkg_name: str, install_hint: str):
    try:
        get_package_share_directory(pkg_name)
        return True
    except PackageNotFoundError:
        raise RuntimeError(
            f"[launch] Missing ROS2 package: '{pkg_name}'.\n"
            f"Install it with:\n  {install_hint}\n"
        )


def generate_launch_description():
    # 1) ensure filter package exists, otherwise fail with install hint
    # _require_pkg(
    #     "imu_filter_madgwick",
    #     "sudo apt update && sudo apt install ros-humble-imu-filter-madgwick",
    # )

    
    # Deprecated pipeline: disabled by default; launch resolves dependencies only
    # when the user explicitly enables the legacy nodes.

    imu_reader = Node(
        package="attitude_sensing_pkg",
        executable="imu_reader_node",   
        name="imu_reader",
        condition=IfCondition(LaunchConfiguration("enable_legacy_imu")),
        output="screen",
        parameters=[
            {
                "port": "/dev/ttyUSB0",
                "baud": 115200,
                "frame_id": "imu_link",
                "input_format": "auto",
                "acc_units": "g",
                "gyro_units": "deg/s",

                "use_rk4_orientation": False,
                "use_sensor_orientation": True,
            }
        ],
        remappings=[
            ("/imu/data_raw", "/imu/data_raw"),
        ],
    )

    madgwick_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        condition=IfCondition(LaunchConfiguration("enable_legacy_imu")),
        output="screen",
        remappings=[
            ("/imu/data_raw", "/imu/data_raw"),
            ("/imu/data", "/imu/data"),
        ],
        # parameters=[...]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_legacy_imu", default_value="false",
            description="Explicitly enable the deprecated IMU reader and filter",
        ),
        imu_reader,
        madgwick_filter,
    ])