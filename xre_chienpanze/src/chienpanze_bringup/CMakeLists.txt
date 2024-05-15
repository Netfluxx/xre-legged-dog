cmake_minimum_required(VERSION 3.8)
project(chienpanze_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(position_publisher src/chienpanze_pos_cmd_pub.cpp)
ament_target_dependencies(position_publisher rclcpp std_msgs)

# Install executable and other directories
install(TARGETS position_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Linting options
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)  # Uncomment if copyright is added and you want to enable linting
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
