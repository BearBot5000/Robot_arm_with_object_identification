cmake_minimum_required(VERSION 3.8)
project(shape_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  shape_detection/shape_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
