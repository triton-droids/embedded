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
        ('share/' + package_name + '/launch', ['launch/motor_control.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='ROS2 Python package for RobStride motor control',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_test = motor_control_node.motor_control_node:main',
            'motor_control_demo = motor_control_node.motor_control:main',
            "motor_control = motor_control_node.controller_node:main",
            "motor_service = motor_control_node.can_node:main"
        ],
    },
)
