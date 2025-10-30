cmake_minimum_required(VERSION 3.5)
project(cyber_is_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
