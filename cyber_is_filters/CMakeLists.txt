cmake_minimum_required(VERSION 3.8)
project(cyber_is_filters)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(line_filter
  src/line_filter_node.cpp
  src/line_filter.cpp
)
ament_target_dependencies(line_filter rclcpp std_msgs)

add_executable(magnet_filter
  src/magnet_filter_node.cpp
  src/magnet_filter.cpp
)
ament_target_dependencies(magnet_filter rclcpp std_msgs)

install(TARGETS
  line_filter
  magnet_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
