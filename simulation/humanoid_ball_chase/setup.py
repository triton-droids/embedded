from glob import glob
from setuptools import find_packages, setup

name = 'humanoid_ball_chase'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml','README.md','VALIDATION.md','requirements.txt']),
                  ('share/'+name+'/assets', glob('assets/*')),
                  ('share/'+name+'/config', glob('config/*.yaml')),
                  ('share/'+name+'/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='Triton Droids', maintainer_email='maxf6.283@gmail.com',
      description='D435 ball perception and pursuit commands in MuJoCo',
      entry_points={'console_scripts':[
          'ball_chase_vision = humanoid_ball_chase.vision_node:main',
          'ball_chase_astar = humanoid_ball_chase.planner_node:main']})
