import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'humanoid_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # mujoco is a pip dependency (requirements.txt); there is no rosdep key for it.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Triton Droids',
    maintainer_email='maxf6.283@gmail.com',
    description='MuJoCo simulation replacing the humanoid hardware peripherals',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mujoco_sim = humanoid_sim.main:main',
            'sim_autostart = humanoid_sim.autostart_node:main',
        ],
    },
)
