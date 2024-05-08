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

include_directories(
  include  # Ensure this directory exists and contains your headers
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/chienpanze_controller_interface.cpp  # Make sure this path is correct
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

  DESTINATION share/${PROJECT_NAME}
)

# Plugin description file export
pluginlib_export_plugin_description_file(controller_interface chienpanze_controller_plugin.xml)  # Adjust this if your plugin description file has a different name or path

ament_package()
