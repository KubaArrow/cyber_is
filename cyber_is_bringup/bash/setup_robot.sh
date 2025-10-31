#!/usr/bin/env bash
# Purpose: bring up Cyber IS (ROS 2 Humble) on boot
set -euo pipefail

# --- ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/victoria/cyber_ws/install/setup.bash

# --- Middleware selection
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- Optional: ROS_DOMAIN_ID if you use multiple robots/networks
# export ROS_DOMAIN_ID=0

# --- Launch
exec ros2 launch cyber_is_bringup is_bringup.launch.py \
  start_description:=false \
  use_gui:=false \
  start_navigation:=true \
  autostart:=true \
  map:=/home/victoria/cyber_is_ws/src/cyber_is/maps/map.yaml \
  start_uart_bridge:=true \
  start_led_controller:=true \
  start_supervisor:=true \
  start_lidar:=true
