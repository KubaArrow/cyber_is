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
# Toggle dynamic SLAM mapping by exporting USE_SLAM=true before running this script.
# When USE_SLAM=true, Nav2 starts without map_server/AMCL and slam_toolbox provides /map.
USE_SLAM=${USE_SLAM:-false}

COMMON_ARGS=(
  start_description:=true
  use_gui:=true
  start_navigation:=true
  autostart:=true
  start_uart_bridge:=true
  start_led_controller:=true
  start_supervisor:=true
  start_lidar:=true
)

if [ "$USE_SLAM" = "true" ]; then
  exec ros2 launch cyber_is_bringup is_bringup.launch.py \
    "${COMMON_ARGS[@]}" \
    use_slam:=true
else
  exec ros2 launch cyber_is_bringup is_bringup.launch.py \
    "${COMMON_ARGS[@]}" \
    map:=/home/is/cyber_ws/src/cyber_is/maps/map.yaml
fi
