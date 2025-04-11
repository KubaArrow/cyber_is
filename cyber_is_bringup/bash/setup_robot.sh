#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

# Uruchom rosbridge_websocket
roslaunch cyber_is_bringup is_bringup.launch
