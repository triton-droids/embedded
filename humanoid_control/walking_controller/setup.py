from setuptools import find_packages, setup

name = 'walking_controller'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Triton Droids', maintainer_email='rul039@ucsd.edu', license='MIT',
    description='Model-based LIPM MPC + TSID walking controller for the simulation stack',
    entry_points={'console_scripts': ['walking_controller = walking_controller.node:main']},
)
