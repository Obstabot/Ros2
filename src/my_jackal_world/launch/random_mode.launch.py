from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    world_file = PathJoinSubstitution([
        FindPackageShare('my_jackal_world'),
        'worlds',
        'simple.world',
    ])

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('jackal_gazebo'),
        'launch',
        'gazebo.launch.py',
    ])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file}.items(),
    )

    obstacle_num_arg = DeclareLaunchArgument(
        'obstacle_num',
        default_value='10',  # 기본값 10개
        description='Number of obstacles to spawn'
    )

    random_spawn_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_jackal_world',
                executable='random_spawn',
                output='screen',
                parameters=[{'obstacle_num': LaunchConfiguration('obstacle_num')}]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(obstacle_num_arg)
    ld.add_action(gazebo_sim)
    ld.add_action(random_spawn_node)

    return ld
