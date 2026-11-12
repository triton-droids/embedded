from glob import glob

from setuptools import find_packages, setup

name = 'humanoid_localization'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/config', glob('config/*')),
        ('share/' + name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Triton Droids', maintainer_email='rul039@ucsd.edu', license='MIT',
    description='Body IMU attitude plus stereo visual odometry (cuVSLAM) base pose estimate',
    entry_points={'console_scripts': [
        'vio_localizer = humanoid_localization.node:main',
        'localization_eval = humanoid_localization.evaluate:main',
        'route_driver = humanoid_localization.route_driver:main',
    ]},
)
