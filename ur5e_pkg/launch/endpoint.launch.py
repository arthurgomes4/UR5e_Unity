import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to the ros_tcp_endpoint package
    ros_tcp_endpoint_dir = get_package_share_directory('ros_tcp_endpoint')

    # declare the ip and port arguements
    ip_arg = DeclareLaunchArgument('ip', default_value='127.0.0.1')
    port_arg = DeclareLaunchArgument('port', default_value='10000')

    # Launch the ros_tcp_endpoint node
    ros_tcp_endpoint_node = Node(
        name='tcp_endpoint',
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        output='screen',
        parameters=[
            {'ROS_IP': LaunchConfiguration('ip')},
            {'ROS_TCP_PORT': LaunchConfiguration('port')}
        ],
        remappings=[
            ('ur5e/joint_command', '/joint_command'),
            ('ur5e/joint_states', '/joint_states')
        ]
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        ros_tcp_endpoint_node
    ])