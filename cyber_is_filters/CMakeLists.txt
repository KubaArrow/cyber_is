cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_filters)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(line_filter
        src/line_filter_node.cpp
        src/line_filter.cpp
)

add_executable(magnet_filter
        src/magnet_filter_node.cpp
        src/magnet_filter.cpp
)

target_link_libraries(line_filter
        ${catkin_LIBRARIES}
)
target_link_libraries(magnet_filter
        ${catkin_LIBRARIES}
)

add_dependencies(line_filter magnet_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
