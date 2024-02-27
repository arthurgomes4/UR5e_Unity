from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch.conditions import UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    is_unity_arg = DeclareLaunchArgument("with_unity", default_value="True")
    is_unity = LaunchConfiguration("with_unity")
    
    with open(os.path.join(get_package_share_directory('ur5e_pkg'),'urdf','ur5e.urdf'), 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': PythonExpression(["True if ", is_unity, " else False"])},
            {'robot_description': robot_desc}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(is_unity)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': PythonExpression(["True if ", is_unity, " else False"])}],
        arguments=['-d', PathJoinSubstitution([FindPackageShare('ur5e_pkg'), 'rviz', 'default.rviz'])]
    )

    return LaunchDescription([
        is_unity_arg,
        robot_state_publisher,
        joint_state_publisher_gui_node,
        rviz_node
    ])