cmake_minimum_required(VERSION 3.5)
project(arm_bot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(joint_state_publisher_gui REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

# Install URDF and launch files
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME}/)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${controller_manager_INCLUDE_DIRS}
  ${ros2_control_INCLUDE_DIRS}
  ${ros2_controllers_INCLUDE_DIRS}
  ${xacro_INCLUDE_DIRS}
  ${robot_state_publisher_INCLUDE_DIRS}
  ${joint_state_publisher_INCLUDE_DIRS}
  ${joint_state_publisher_gui_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${joint_trajectory_controller_INCLUDE_DIRS}
  ${joint_state_broadcaster_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF and config files
install(DIRECTORY
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Add tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
