from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_jackal_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', ['worlds/simple.world']),
        ('share/'+ package_name + '/models',[
          'models/cylinder.sdf',
          'models/marker.sdf',
          'models/turtlebot3_waffle.urdf'
        ]),
        ('share/' + package_name + '/msg',glob('src/my_jackal_world/msg/*.msg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunbin',
    maintainer_email='eunbin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'random_spawn = my_jackal_world.random_spawn:main',
          'random_dynamic_spawn = my_jackal_world.dynamics_obstacles.random_dynamic_spawn:main',
          'turtlebot_controller = my_jackal_world.dynamics_obstacles.turtlebot_controller:main',
          'multi_controller_launcher = my_jackal_world.dynamics_obstacles.multi_controller_launcher:main',
          'central_reset_node = my_jackal_world.dynamics_obstacles.central_reset_node:main',
          'astar_planner_node = my_jackal_world.path.astar_planner_node:main',
          'follower_node = my_jackal_world.path.follower_node:main'
        ],
    },
)
