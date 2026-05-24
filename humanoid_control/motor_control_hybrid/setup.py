from setuptools import setup
from glob import glob
import os

package_name = 'motor_control_hybrid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'grpcio',
        'protobuf',
        'robstride-dynamics',
        'websockets',
    ],
    zip_safe=True,
    maintainer='rcli',
    maintainer_email='rul039@ucsd.edu',
    description='Hybrid motor control: Python CAN + C++ Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_can_node = motor_control_hybrid.python_can_node:main',
            'motor_sdk_gateway_node = motor_control_hybrid.motor_sdk_gateway_node:main',
            'fake_motor_node = motor_control_hybrid.fake_motor_node:main',
            'double_pendulum_websocket_node = motor_control_hybrid.double_pendulum_websocket_node:main',
            'policy_bridge_node = motor_control_hybrid.policy_bridge_node:main',
        ],
    },
)
