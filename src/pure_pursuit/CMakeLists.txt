cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  tf2_ros
  tf2)

  find_package(${PACKAGE} REQUIRED)

endforeach()


# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)

ament_target_dependencies(pure_pursuit
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2 eigen3_cmake_module Eigen3
)

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)

ament_target_dependencies(waypoint_visualizer
  rclcpp geometry_msgs std_msgs visualization_msgs eigen3_cmake_module Eigen3
)

# Install Cpp executables
install(TARGETS
  pure_pursuit
  waypoint_visualizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
