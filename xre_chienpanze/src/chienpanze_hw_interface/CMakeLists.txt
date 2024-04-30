cmake_minimum_required(VERSION 3.5)
project(chienpanze_hw_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  hardware_interface
  pluginlib
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  include/pi3hat/pi3hat.cc           # Correct path inside the src folder
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Plugins are declared here
pluginlib_export_plugin_description_file(hardware_interface chienpanze_hw_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
