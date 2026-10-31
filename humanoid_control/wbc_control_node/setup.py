from glob import glob
from setuptools import find_packages, setup

name = 'wbc_control_node'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Triton Droids', maintainer_email='maxf6.283@gmail.com', license='MIT',
    description='TSID policy-to-torque bridge for the existing simulation stack',
    entry_points={'console_scripts': ['wbc_control_node = wbc_control_node.node:main']},
)
