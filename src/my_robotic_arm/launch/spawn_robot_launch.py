from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='/root/ros2_1/src/my_robotic_arm/urdf/robot_arm.urdf',
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'x', default_value='0', description='X position of the robot'
        ),
        DeclareLaunchArgument(
            'y', default_value='0', description='Y position of the robot'
        ),
        DeclareLaunchArgument(
            'z', default_value='0.5', description='Z position of the robot'
        ),
        DeclareLaunchArgument(
            'R', default_value='0', description='Roll orientation of the robot'
        ),
        DeclareLaunchArgument(
            'P', default_value='0', description='Pitch orientation of the robot'
        ),
        DeclareLaunchArgument(
            'Y', default_value='0', description='Yaw orientation of the robot'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot_arm', 
                '-file', LaunchConfiguration('model'), 
                '-x', LaunchConfiguration('x'), 
                '-y', LaunchConfiguration('y'), 
                '-z', LaunchConfiguration('z'), 
                '-R', LaunchConfiguration('R'), 
                '-P', LaunchConfiguration('P'), 
                '-Y', LaunchConfiguration('Y')
            ],
            output='screen'
        ),
    ])
