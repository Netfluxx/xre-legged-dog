cmake_minimum_required(VERSION 3.5)
project(chienpanze_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)


ament_package()
