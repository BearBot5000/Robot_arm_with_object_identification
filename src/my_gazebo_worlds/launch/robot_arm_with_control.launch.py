from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'robot_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('my_gazebo_worlds'),
        'urdf',
        urdf_file_name)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Revolute_9_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Revolute_10_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Revolute_11_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Revolute_12_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Revolute_13_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Slider_14_position_controller'],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['Slider_15_position_controller'],
            output='screen'),
    ])
