# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():

#     world_file = PathJoinSubstitution([
#         FindPackageShare('my_jackal_world'),
#         'worlds',
#         'simple.world',  # ← 여기에 원하는 world 파일명
#     ])

#     gazebo_launch = PathJoinSubstitution([
#         FindPackageShare('jackal_gazebo'),
#         'launch',
#         'gazebo.launch.py',  # ← 기존 general launch 파일 재사용
#     ])

#     gazebo_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([gazebo_launch]),
#         launch_arguments={'world_path': world_file}.items(),
#     )

#     ld = LaunchDescription()
#     ld.add_action(gazebo_sim)


#     return ld
