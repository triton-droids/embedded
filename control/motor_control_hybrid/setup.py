from setuptools import setup
from glob import glob
import os

package_name = 'motor_control_hybrid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/models/20260914_170827',
         glob('models/20260914_170827/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='rcli',
    maintainer_email='rul039@ucsd.edu',
    description='Simulation control: C++ scheduling, ONNX policy and fake motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_motor_node = motor_control_hybrid.fake_motor_node:main',
            'tracking_policy_node = motor_control_hybrid.tracking_policy_node:main',
            'joint_state_monitor_node = motor_control_hybrid.joint_state_monitor_node:main',
        ],
    },
)
