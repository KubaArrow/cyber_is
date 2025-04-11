cmake_minimum_required(VERSION 3.0.2)
project(uart_bridge)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)

catkin_package(
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(log
        src/log.c
)

add_library(serial_slip
        src/serial_slip.c
)
target_link_libraries(serial_slip
        log
)


add_executable(uart_bridge src/main.cpp)
target_link_libraries(uart_bridge
        log
        serial_slip
        ${catkin_LIBRARIES}
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

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
