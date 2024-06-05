cmake_minimum_required(VERSION 3.8)
project(my_gazebo_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(effort_controllers REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Install URDF and launch files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
