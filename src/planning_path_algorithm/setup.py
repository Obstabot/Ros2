from setuptools import setup
from setuptools import setup, find_packages

package_name = 'planning_path_algorithm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leunbin',
    maintainer_email='leunbin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_node = planning_path_algorithm.astar.astar_node:main',
        ],
    },
)
