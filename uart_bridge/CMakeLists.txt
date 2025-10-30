cmake_minimum_required(VERSION 3.8)
project(uart_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
)

add_library(log STATIC
  src/log.c
)

add_library(serial_slip STATIC
  src/serial_slip.c
)
target_link_libraries(serial_slip
  log
  pthread
)

add_executable(uart_bridge
  src/main.cpp
)
ament_target_dependencies(uart_bridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
)
target_link_libraries(uart_bridge
  log
  serial_slip
  pthread
)

target_compile_definitions(log PRIVATE
  WS_BRIDGE_LOG_LEVEL=LOG_TRACE
  LOG_USE_COLOR
  USE_HIGHRES_TIME
)

target_compile_definitions(uart_bridge PRIVATE
  WS_BRIDGE_LOG_LEVEL=LOG_TRACE
  LOG_USE_COLOR
  USE_HIGHRES_TIME
)

install(TARGETS
  log
  serial_slip
  uart_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
