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
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'robstride-dynamics',
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
            'target_gateway_node = motor_control_hybrid.target_gateway_node:main',
        ],
    },
)
