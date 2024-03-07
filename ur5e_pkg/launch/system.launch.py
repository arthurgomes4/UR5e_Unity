from  launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ur_moveit_file = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])
    ur_description_file = PathJoinSubstitution([FindPackageShare('ur5e_pkg'), 'urdf', 'ur5e.urdf'])

    ros2_control_file = PathJoinSubstitution([FindPackageShare('ur5e_pkg'), 'launch', 'control.launch.py'])

    endpoint_file = PathJoinSubstitution([FindPackageShare('ur5e_pkg'), 'launch', 'endpoint.launch.py'])

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_file),
        launch_arguments={
            'use_sim_time': "true",
            'description_file': ur_description_file,
            'use_fake_hardware' : "true",
            'ur_type': 'ur5e'
        }.items(),
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_file),
    )

    endpoint_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(endpoint_file)
    )

    return LaunchDescription([
        endpoint_file,
        ros2_control_launch,
        ur_moveit_launch
        ])