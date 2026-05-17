from setuptools import setup
from glob import glob
import os

package_name = 'motor_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),

        # 安装所有 launch 文件（不只 motor_control.launch.py）
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # 如果你把 setup_can0.sh 也放在 launch/ 里，需要一起安装
        ('share/' + package_name + '/launch', glob('launch/*.sh')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='ROS2 Python package for RobStride motor control + IMU DR',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_test = motor_control_node.motor_control_node:main',
            'motor_control_demo = motor_control_node.motor_control:main',
            'motor_control = motor_control_node.controller_node:main',
            'motor_service = motor_control_node.can_node:main',
            'imu_dr_rk4_node = motor_control_node.imu_dr_rk4_node:main',
            'swing_leg_node = motor_control_node.swing_leg_node:main',
        ],
    },
)
