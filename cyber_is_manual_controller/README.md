
# Manual Mode (ROS 2 Humble)

This ROS 2 (Humble) package enables **manual control mode** for a mobile robot. It provides:

- Joystick teleoperation (`sensor_msgs/Joy` -> `geometry_msgs/Twist`) in `rclpy`.
- Optional USB camera streaming via `mjpg_streamer`.

## 🧰 Features

- rclpy joystick teleop with configurable axes, scales and deadman.
- Sensor-data QoS for robust input from `joy` driver.
- Optional `mjpg_streamer` runner.

---

## 📦 Installation

1. Place this package in your ROS 2 workspace `src` and install dependencies:

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-joy
```

2. (Optional) Install MJPG-Streamer if you want camera streaming:

```bash
sudo apt update
sudo apt install cmake libjpeg8-dev
cd ~/ros2_ws/src/cyber_is/cyber_is_manual_controller/
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
```

3. Build with colcon and source:

```bash
cd ~/ros2_ws
colcon build --packages-select cyber_is_manual_controller
source install/setup.bash
```

---

## ▶️ Usage

Start the ROS 2 `joy` driver (adjust device as needed):

```bash
ros2 run joy joy_node
```

Launch manual mode (joystick teleop; optional streamer):

```bash
ros2 launch cyber_is_manual_controller start_manual_mode.launch.py start_streamer:=false
```

- Teleop publishes `Twist` to `/cmd_vel` by default.
- To change mappings/scales:

```bash
ros2 launch cyber_is_manual_controller start_manual_mode.launch.py \
  axis_linear:=1 axis_angular:=0 scale_linear:=0.5 scale_angular:=1.0 deadman_button:=-1 \
  output_topic:=/cmd_vel joy_topic:=/joy
```

If `start_streamer:=true`, MJPG-Streamer starts. Stream is available at:

```
http://<robot_ip>:8080/stream
```

---

## 📝 Notes

- MJPG-Streamer expected path after build:
  `cyber_is_manual_controller/mjpg-streamer/mjpg-streamer-experimental/`
- The USB camera is expected at `/dev/video0`. Adjust `bash/start_streamer.sh` as needed.

---

## 📁 Directory Structure

```
cyber_is_manual_controller/
├── launch/
│   └── start_manual_mode.launch.py
├── bash/
│   └── start_streamer.sh
├── src/cyber_is_manual_controller/
│   ├── __init__.py
│   ├── joystick_node.py
│   └── streamer.py
├── resource/
│   └── cyber_is_manual_controller
├── setup.py
├── setup.cfg
└── package.xml
```

---

## 🧪 Tested On

- Ubuntu 22.04
- ROS 2 Humble
- MJPG-Streamer (UVCCapture plugin)

---
