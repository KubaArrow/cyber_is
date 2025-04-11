#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
source /home/pi/catkin_ws/src/cyber_is_bringup/bash/env.sh

# Uruchom roscore w tle
roscore &
sleep 5  # poczekaj aż roscore się w pełni uruchomi

# Uruchom rosbridge_websocket
roslaunch cyber_is_bringup is_bringup.launch
