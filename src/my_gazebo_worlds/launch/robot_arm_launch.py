# robot_arm_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_gazebo_worlds = get_package_share_directory('my_gazebo_worlds')
    pkg_my_robot_arm_controller = get_package_share_directory('my_robot_arm_controller')

    world = os.path.join(pkg_my_gazebo_worlds, 'worlds', 'my_world.world')
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_my_gazebo_worlds, 'urdf', 'robot_arm_simple.urdf.xacro'])
    ])
    robot_controllers = os.path.join(pkg_my_gazebo_worlds, 'config', 'robot_controllers_simple.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': world}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_controllers],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller'],
            output='screen'
        ),
        Node(
            package='my_robot_arm_controller',
            executable='my_robot_arm_controller',
            output='screen'
        )
    ])
