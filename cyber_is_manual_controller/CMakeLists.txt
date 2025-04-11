cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_manual_controller)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
        scripts/start_streamer_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

