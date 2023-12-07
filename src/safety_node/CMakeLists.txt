cmake_minimum_required(VERSION 3.5)
project(safety_node)

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
  ament_cmake_python
  rclcpp
  rclpy
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/safety_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
