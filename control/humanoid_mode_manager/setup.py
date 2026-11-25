import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'humanoid_mode_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rcli',
    maintainer_email='rul039@ucsd.edu',
    description='Robot mode state machine',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['mode_manager = humanoid_mode_manager.ros.node:main']},
)
