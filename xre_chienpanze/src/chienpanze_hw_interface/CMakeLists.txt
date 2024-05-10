cmake_minimum_required(VERSION 3.5)
project(chienpanze_hw_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  include/pi3hat/pi3hat.cc
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
