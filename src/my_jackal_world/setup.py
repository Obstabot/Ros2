from setuptools import setup

package_name = 'my_jackal_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_world.launch.py']),
        # ✅ world 폴더도 포함 (필요하다면)
        ('share/' + package_name + '/worlds', ['worlds/simple.world']),
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
        ],
    },
)
