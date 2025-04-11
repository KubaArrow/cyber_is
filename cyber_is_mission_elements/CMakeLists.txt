cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_mission_elements)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
)

## Deklaracja pakietu
catkin_package()

## Ścieżki do nagłówków
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Kompilacja node'a
add_executable(mission_collector_node src/mission_collector_node.cpp)

target_link_libraries(mission_collector_node
        ${catkin_LIBRARIES}
)

add_dependencies(mission_collector_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)