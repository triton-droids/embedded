from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'motor_control_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='maintainer@example.com',
    description='ROS2 Python package for RobStride motor control',
    license='TODO',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'motor_control = motor_control_node.motor_control:main',
        ],
    },
)
