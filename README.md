# Cyber IS (ROS 2 Humble)

A set of ROS 2 packages to run the Cyber IS robot on Ubuntu 22.04 (ROS 2 Humble). This guide provides complete installation, build, and run instructions for the target robot, including a systemd option.

- Packages included: `cyber_is_bringup`, `cyber_is_description`, `cyber_is_led_controller`, `cyber_is_manual_controller`, `cyber_is_navigation`, `cyber_is_supervisor`, `cyber_is_filters`, `uart_bridge`.
- Build and run: `colcon build`, `ros2 launch ...`.

## Requirements and Dependencies

- OS: Ubuntu 22.04 (Jammy), recommended RPi 4 or x86_64.
- ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html
- DDS (RMW):
  - Recommended: Cyclone DDS. Install: `sudo apt install ros-humble-rmw-cyclonedds-cpp` and set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
  - Alternative: Fast DDS (available by default). Use `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` or unset the variable.

### Minimal apt packages
```bash
sudo apt update && sudo apt install -y \
  ros-humble-rclcpp ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs \
  ros-humble-launch ros-humble-launch-ros \
  ros-humble-robot-state-publisher ros-humble-urdf ros-humble-xacro ros-humble-rviz2 \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros \
  ros-humble-nav2-bringup ros-humble-nav2-costmap-2d ros-humble-nav2-planner ros-humble-nav2-controller \
  ros-humble-nav2-bt-navigator ros-humble-nav2-behaviors ros-humble-nav2-lifecycle-manager ros-humble-nav2-amcl ros-humble-nav2-map-server \
  ros-humble-joy \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosbridge-server ros-humble-rosapi \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-urdf \ 
  ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-tf2-geometry-msgs ros-humble-tf-transformations \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rviz2 \
  ros-humble-nav2-behaviors
```

Add the user to the `dialout` group for access to `/dev/ttyACM0`:
```bash
sudo usermod -a -G dialout $USER
# log out and log back in
```

## Workspace Layout and Build

Example layout on the robot:
```bash
mkdir -p ~/cyber_is_ws/src
cd ~/cyber_is_ws/src
git clone https://github.com/KubaArrow/cyber_is.git
cd ..

# (optional) clean environment
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash
```

If CMake cache causes trouble: `colcon build --cmake-clean-cache` or remove `build/ install/ log/`.

## Quick Start (all-in-one bringup)

Bringup launches key nodes (UART bridge, supervisor, LED controller, Nav2, and optionally robot description + RViz2).

Full bringup example:
```bash
source /opt/ros/humble/setup.bash
source ~/cyber_is_ws/install/setup.bash

ros2 launch cyber_is_bringup is_bringup.launch.py \
  start_description:=false use_gui:=false \
  start_navigation:=true autostart:=true map:=/path/to/map.yaml \
  start_uart_bridge:=true \
  start_led_controller:=true \
  start_supervisor:=true
```

Default parameter files used by bringup:
- UART bridge: `cyber_is_bringup/config/uart_bridge.yaml` (port, frequency, topic names).
- LED controller: `cyber_is_bringup/config/leds_controller.yaml`.
- Supervisor: `cyber_is_bringup/config/supervisor.yaml`.
- Nav2: `cyber_is_navigation/config/nav2_params.yaml` (requires a map YAML path).

Launch robot description + RViz2 only:
```bash
ros2 launch cyber_is_description display.launch.py use_gui:=false
```

## Packages and How to Run

- cyber_is_bringup
  - Main launch: `cyber_is_bringup/launch/is_bringup.launch.py` with flags to enable/disable modules.
- uart_bridge
  - C++ node bridging ROS 2 and a microcontroller over SLIP on a serial port.
  - Default topics: `/cmd_vel` (sub), `/leds` (sub), `/low_level_odom`, `/imu`, `/magnet`, `/line_detector`, `/battery`, `/status_topic` (pub).
  - Important: set the correct port (`/dev/ttyACM0`) and ensure user is in `dialout`.
  - Standalone: `ros2 launch uart_bridge start_bridge.launch.py` (also covered by bringup params).
- cyber_is_supervisor
  - rclcpp node managing MANUAL/AUTO modes. Subscribes `/robot_mode` (String), publishes `/robot_state`, `/leds_mode`, `/supervisor/heartbeat`.
  - Standalone: `ros2 launch cyber_is_supervisor supervisor.launch.py`.
  - Change mode: `ros2 topic pub /robot_mode std_msgs/String '{data: "AUTO"}' -1`.
- cyber_is_led_controller
  - Python node controlling LEDs. Subscribes `/leds_mode`, publishes colors to `/leds`.
  - Standalone: `ros2 launch cyber_is_led_controller led_controller.launch.py`.
- cyber_is_manual_controller
  - Joystick teleop: `sensor_msgs/Joy` -> `geometry_msgs/Twist`.
  - Start the joystick driver: `ros2 run joy joy_node`, then: `ros2 launch cyber_is_manual_controller start_manual_mode.launch.py`.
- cyber_is_navigation
  - Nav2 in a single composition container (map_server, amcl, planner, controller, behaviors, bt_navigator + lifecycle manager).
  - Standalone: `ros2 launch cyber_is_navigation start_navigation.launch.py map:=/path/to/map.yaml`.
- cyber_is_filters
  - C++ filters for line and magnetometer. Standalone: `ros2 launch cyber_is_filters start_filters.launch.py`.
- cyber_is_description
  - URDF/Xacro + RViz2. `ros2 launch cyber_is_description display.launch.py`.

## Systemd (start on boot)

Example systemd unit to start the ROS 2 bringup (adjust paths and user):
```ini
[Unit]
Description=Cyber IS Bringup (ROS 2 Humble)
After=network-online.target time-sync.target
Wants=network-online.target

[Service]
Type=simple
User=is
WorkingDirectory=/home/is/cyber_ws
# Daj chwilę na podniesienie się sieci/USB
ExecStartPre=/bin/sleep 3
ExecStart=/home/is/cyber_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh
Restart=always
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=30
StandardOutput=journal
StandardError=journal
Environment=LANG=C.UTF-8
Environment=LC_ALL=C.UTF-8
# Opcjonalnie: jeżeli LIDAR pojawia się jako /dev/ttyUSB0 i potrzebujesz czekać
# After=dev-ttyUSB0.device

[Install]
WantedBy=multi-user.target

```

Activation:
```bash
sudo nano /etc/systemd/system/cyber-is-bringup.service  # paste and adjust the template above
chmod +x /home/is/cyber_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh

sudo systemctl daemon-reload
sudo systemctl enable cyber-is-bringup.service
sudo systemctl start cyber-is-bringup.service
sudo systemctl status cyber-is-bringup.service
```

Note: `cyber_is_bringup/services/robot_bringup.service` in the repo was meant for ROS 1. Use the ROS 2 template above instead.

## Network (optional static IP)

If you want a static Wi‑Fi IP for the robot, a netplan example (adjust SSID/password and addresses):
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"
      dhcp4: no
      addresses: [192.168.1.11/24]
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```
```bash
sudo netplan apply
```

## Troubleshooting

- CMake error: “Could not find ROS middleware implementation 'rmw_cyclonedds_cpp'”
  - Install `sudo apt install ros-humble-rmw-cyclonedds-cpp` and set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, or switch to Fast DDS: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH warnings referencing non-existent `install/...`
  - Open a fresh shell and `source /opt/ros/humble/setup.bash` before building. Clear `build/ install/ log/` if needed.
- No permission to `/dev/ttyACM0`
  - Add user to `dialout`: `sudo usermod -a -G dialout $USER` and re-login.
- RViz2/joint_state_publisher_gui missing
  - Install: `sudo apt install ros-humble-joint-state-publisher-gui`. Launch with `use_gui:=false` if not needed.
- Nav2 does not load the map
  - Pass a full path to the map `.yaml` in `map:=...`. Verify file permissions.

## Quick Checks

- LED controller: `ros2 topic pub /leds_mode std_msgs/String '{"data": "SIDE_GREEN_BREATH"}' -1`
- Supervisor: `ros2 topic pub /robot_mode std_msgs/String '{"data": "MANUAL"}' -1`
- Joystick: `ros2 run joy joy_node` and `ros2 launch cyber_is_manual_controller start_manual_mode.launch.py`
- UART bridge: `ros2 topic echo /low_level_odom`, `ros2 topic echo /imu`

## Dev Notes

- Stack style: ROS 2 Humble, `colcon`, Python (ament_python) and C++ (ament_cmake).
- Key topics: `/cmd_vel`, `/low_level_odom`, `/imu`, `/magnet`, `/line_detector`, `/leds`, `/leds_mode`, `/robot_state`, `/robot_mode`.
- Parameter files live under `cyber_is_bringup/config` and `cyber_is_navigation/config`.
