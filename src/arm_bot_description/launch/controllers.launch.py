from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('arm_bot_description')

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager_node',
        parameters=[
            os.path.join(share_dir, 'config', 'arm_bot_control.yaml')
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])
