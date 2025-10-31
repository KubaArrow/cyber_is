#!/usr/bin/env bash
#!/usr/bin/env bash
# robust: ERR + pipefail, ale BEZ -u podczas source
set -Eeo pipefail

# --- tymczasowo wyłącz -u, bo setup.bash używa nieistniejących zmiennych
set +u
# --- ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/is/cyber_ws/install/setup.bash

# --- Middleware selection
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- Optional: ROS_DOMAIN_ID if you use multiple robots/networks
export ROS_DOMAIN_ID=2

# --- Launch
exec ros2 launch cyber_is_bringup is_bringup.launch.py \
  start_description:=true \
  use_gui:=false \
  start_navigation:=true \
  autostart:=true \
  map:=/home/is/cyber_ws/src/cyber_is/maps/map.yaml \
  start_uart_bridge:=true \
  start_led_controller:=true \
  start_supervisor:=true \
  start_lidar:=true
