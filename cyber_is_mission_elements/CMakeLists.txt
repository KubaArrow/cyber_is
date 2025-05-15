cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_mission_elements)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## 1) MissionCollector
add_library(mission_collector
        src/MissionCollector.cpp
)
target_link_libraries(mission_collector
        ${catkin_LIBRARIES}
)

## 2) SearchStart
add_library(search_start
        src/SearchStart.cpp
)
target_link_libraries(search_start
        ${catkin_LIBRARIES}
)

## 3) SearchOrientation
add_library(search_orientation
        src/SearchOrientation.cpp
)
target_link_libraries(search_orientation
        ${catkin_LIBRARIES}
)

## 4) ZoneNavigator
add_library(zone_navigator
        src/ZoneNavigator.cpp
)
target_link_libraries(zone_navigator
        ${catkin_LIBRARIES}
)

## 5) SearchMeta
add_library(search_meta
        src/SearchMeta.cpp
)
target_link_libraries(search_meta
        ${catkin_LIBRARIES}
)

## 6) MissionController
add_library(mission_controller
        src/MissionController.cpp
)
target_link_libraries(mission_controller
        ${catkin_LIBRARIES}
        mission_collector
        search_start
        search_orientation
        zone_navigator
        search_meta
)

## 7) Node executable
add_executable(mission_controller_node
        src/main.cpp
)
target_link_libraries(mission_controller_node
        ${catkin_LIBRARIES}
        mission_controller
)

add_dependencies(mission_controller_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
