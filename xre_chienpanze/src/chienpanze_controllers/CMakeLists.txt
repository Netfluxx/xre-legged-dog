cmake_minimum_required(VERSION 3.5)
project(chienpanze_controller_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Uncomment the following section to add dependencies to other ROS 2 packages
# find_package(other_ros2_package REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
)

# Add library
add_library(${PROJECT_NAME}
  src/chienpanze_controller_interface.cpp # Adjust this if your file name is different
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files and other resources
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# If using pluginlib, export the plugin
pluginlib_export_plugin_description_file(chienpanze_controller_interface chienpanze_controller_plugin.xml)

# Uncomment if you have executable nodes
# add_executable(node_name src/node_file.cpp)
# target_link_libraries(node_name ${PROJECT_NAME})
# ament_target_dependencies(node_name ${dependencies})
# install(TARGETS node_name DESTINATION lib/${PROJECT_NAME})

ament_package()
