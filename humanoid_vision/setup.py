from setuptools import find_packages, setup

package_name = 'humanoide_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darin',
    maintainer_email='rul039@ucsd.edu',
    description='Display RealSense color and aligned depth images side by side.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = humanoide_vision.vision_node:main'
        ],
    },
)
