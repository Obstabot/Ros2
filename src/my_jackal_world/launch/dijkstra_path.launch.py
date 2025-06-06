from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 0. 인자 (장애물 개수)
    obstacle_arg = DeclareLaunchArgument(
        'obstacle_num', default_value='10', description='Number of obstacles')
    obstacle_num = LaunchConfiguration('obstacle_num')

    pkg_share = get_package_share_directory('my_jackal_world')

    # 1. Gazebo + Jackal + 랜덤 장애물
    random_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'random_mode.launch.py')),
        launch_arguments={'obstacle_num': obstacle_num}.items())

    # 2. dijkstra 플래너  (console_scripts 이름 사용!)
    planner = Node(
        package='my_jackal_world',
        executable='dijkstra_planner_node',   # dijkstra_planner.py 가 아님!
        name='dijkstra_planner',
        output='screen')

    # 3. Pure-Pursuit 팔로워
    follower = Node(
        package='my_jackal_world',
        executable='follower_node',        # follower_node.py
        name='path_follower',
        output='screen',
        parameters=[
            {'lookahead': 0.4,
             'linear_speed': 0.3,
             'target_tolerance':0.25}
        ])

    return LaunchDescription([obstacle_arg, random_mode, planner, follower])
