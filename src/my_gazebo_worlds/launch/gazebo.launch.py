from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"))

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF directly
    robot_description_content = Command(
        ["cat ", PathJoinSubstitution([FindPackageShare("my_gazebo_worlds"), "urdf", "robot_arm_simple.urdf"])]
    )
    robot_description = {"robot_description": robot_description_content}

    # Load robot controllers configuration from YAML file
    control_params = PathJoinSubstitution([FindPackageShare("my_gazebo_worlds"), "config", "robot_controllers_simple.yaml"])

    return LaunchDescription([
        # Start Gazebo server and client
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "robot_arm", "-file", PathJoinSubstitution([FindPackageShare("my_gazebo_worlds"), "urdf", "robot_arm_simple.urdf"])],
            output="screen",
        ),
        # Start robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}]
        ),
        # Start ros2_control node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[control_params, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        # Start joint_state_broadcaster after spawning the entity
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "robot_arm", "-file", PathJoinSubstitution([FindPackageShare("my_gazebo_worlds"), "urdf", "robot_arm_simple.urdf"])],
                    output="screen",
                ),
                on_exit=[Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen",
                )]
            )
        ),
        # Start forward_position_controller after joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                on_exit=[Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                )]
            )
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
