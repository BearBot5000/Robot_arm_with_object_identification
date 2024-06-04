from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shape_detection',
            executable='shape_detection_node',
            name='shape_detection_node',
            output='screen',
        ),
    ])
