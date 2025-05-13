cmake_minimum_required(VERSION 3.5)
project(virtual_costmap_layer VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

##############################################################################
# Pakiety
find_package(catkin REQUIRED COMPONENTS          # <-- MUSI być COMPONENTS
        roscpp
        costmap_2d
        dynamic_reconfigure
        message_generation
        geometry_msgs
        tf
)

##############################################################################
# dynamic_reconfigure + messages  (PRZED catkin_package!)
generate_dynamic_reconfigure_options(
        cfg/VirtualLayer.cfg
)

add_message_files(
        FILES
        Form.msg
        Obstacles.msg
        Zone.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

##############################################################################
# catkin_package
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES virtual_layer
        CATKIN_DEPENDS
        roscpp
        costmap_2d
        dynamic_reconfigure
        message_runtime
        geometry_msgs
        tf
)

##############################################################################
# Kompilacja
include_directories(
        include
        ${catkin_INCLUDE_DIRS}             # zawiera <devel>/include
)

add_library(virtual_layer
        src/virtual_layer.cpp
)

target_link_libraries(virtual_layer
        ${catkin_LIBRARIES}
)

add_dependencies(virtual_layer       # zagwarantuj, że nagłówki są gotowe
        ${PROJECT_NAME}_gencfg             #  nagłówki z dynamic_reconfigure
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}         #  nagłówki z .msg
)
