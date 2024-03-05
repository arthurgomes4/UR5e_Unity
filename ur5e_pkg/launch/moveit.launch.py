from  launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ur_moveit_file = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])
    ur_description_file = PathJoinSubstitution([FindPackageShare('ur5e_pkg'), 'urdf', 'ur5e.urdf'])

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_file),
        launch_arguments={
            'use_sim_time': "true",
            'description_file': ur_description_file,
            'use_fake_hardware' : "true",
            'ur_type': 'ur5e'
        }.items(),
    )

    return LaunchDescription([
        ur_moveit_launch
        ])