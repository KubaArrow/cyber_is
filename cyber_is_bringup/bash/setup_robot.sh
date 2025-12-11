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

# Headless-safe GUI toggle: disable GUI if no DISPLAY unless overridden via USE_GUI
if [ -z "${USE_GUI+x}" ]; then
  if [ -n "${DISPLAY:-}" ]; then
    USE_GUI=true
  else
    USE_GUI=false
  fi
fi

if [ -z "${MAP_FILE:-}" ]; then
  NAV_SHARE=$(ros2 pkg prefix -q cyber_is_navigation 2>/dev/null | head -n 1)
  if [ -n "$NAV_SHARE" ] && [ -f "$NAV_SHARE/share/cyber_is_navigation/maps/map.yaml" ]; then
    MAP_FILE="$NAV_SHARE/share/cyber_is_navigation/maps/map.yaml"
  elif [ -f "/home/is/cyber_ws/src/cyber_is/cyber_is_navigation/maps/map.yaml" ]; then
    MAP_FILE="/home/is/cyber_ws/src/cyber_is/cyber_is_navigation/maps/map.yaml"
  else
    echo "Warning: default map.yaml not found. Pass MAP_FILE=/path/to/map.yaml" >&2
    MAP_FILE=''
  fi
fi

if [ -n "$MAP_FILE" ]; then
  MAP_ARG=(map:=${MAP_FILE})
else
  MAP_ARG=()
fi

COMMON_ARGS=(
  start_description:=true
  use_gui:=${USE_GUI}
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
    "${MAP_ARG[@]}"
fi
