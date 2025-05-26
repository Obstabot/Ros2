from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('my_jackal_world')

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'simple.world'
    ])

    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('jackal_gazebo'),
        'launch',
        'gazebo.launch.py'
    ])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world_path': world_file,
            'extra_gazebo_args': '--verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so'
            }.items()
    )

    dynamic_spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_jackal_world',
                executable='random_dynamic_spawn',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    controllers = []
    for i in range(1, 3):
        controllers.append(
            TimerAction(
                period=2.0*i,
                actions=[
                    Node(
                        package='my_jackal_world',
                        executable='turtlebot_controller',
                        name = f'turtlebot_controller_{i}',
                        arguments=[f'turtlebot_{i}'],
                        output='screen',
                        parameters=[{'use_sim_time': True}]   
                    )
                ]
            )
        )
    
    # animated_obstacles = TimerAction(
    #     period=10.0,
    #     actions=controllers
    # )


    ld = LaunchDescription()
    ld.add_action(gazebo_sim)
    ld.add_action(dynamic_spawn)
    for ctrl in controllers:
        ld.add_action(ctrl)
    # ld.add_action(animated_obstacles)
    return ld
