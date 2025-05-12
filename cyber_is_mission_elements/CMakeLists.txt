cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_mission_elements)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        geometry_msgs
        tf
        actionlib
        move_base_msgs
        std_srvs
        virtual_costmap_layer
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

add_executable(search_start_node src/search_start_node.cpp)

target_link_libraries(search_start_node
        ${catkin_LIBRARIES}

)

add_executable(search_meta_node src/search_meta_node.cpp)

target_link_libraries(search_meta_node
        ${catkin_LIBRARIES}
)

add_executable(search_orientation src/search_orientation.cpp)

target_link_libraries(search_orientation
        ${catkin_LIBRARIES}
)

add_executable(zone_navigator src/zone_navigator.cpp)

target_link_libraries(zone_navigator
        ${catkin_LIBRARIES}
)

add_dependencies(mission_collector_node search_start_node search_meta_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)