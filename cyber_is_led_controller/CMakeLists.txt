cmake_minimum_required(VERSION 3.0.2)
project(cyber_is_led_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()
catkin_python_setup()


catkin_install_python(PROGRAMS
  scripts/leds_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)