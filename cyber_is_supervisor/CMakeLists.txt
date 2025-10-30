cmake_minimum_required(VERSION 3.8)
project(cyber_is_supervisor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(supervisor_node src/supervisor_node.cpp)
ament_target_dependencies(supervisor_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
