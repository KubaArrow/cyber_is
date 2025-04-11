
# Manual Mode ROS Package

This ROS package enables **manual control mode** for a mobile robot in ROS Noetic. It provides:

- Streaming from a USB camera via `mjpg_streamer`.
- Forwarding of velocity commands from `/cmd_vel_app` to `/cmd_vel`.

## 🧰 Features

- Launches `mjpg_streamer` with the desired resolution and framerate.
- Forwards velocity commands using `topic_tools/relay`.
- Launchable via a single ROS launch file.

---

## 📦 Installation

1. **Clone this repository (or copy this package) into your ROS workspace:**

```bash
cd ~/catkin_ws/src
git clone <your repository url> manual_mode
```

2. **Install MJPG-Streamer:**

```bash
sudo apt update
sudo apt install cmake libjpeg8-dev
cd ~/catkin_ws/src/cyber_is/cyber_is_manual_controller/
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
```

3. **Make the stream launcher script executable:**

```bash
chmod +x ~/catkin_ws/src/cyber_is/cyber_is_manual_controller/bash/start_streamer.sh
```

4. **Build the workspace:**

```bash
cd ~/catkin_ws
catkin_make
```

5. **Source the workspace:**

```bash
source devel/setup.bash
```

---

## ▶️ Usage

To start manual mode (camera stream + velocity relay), run:

```bash
roslaunch manual_mode manual_mode.launch
```

- The MJPG video stream will be available at:  
  `http://<robot_ip>:8080/stream`

- The `/cmd_vel_app` topic will be relayed to `/cmd_vel` for manual driving.

---

## 📝 Notes

- MJPG-Streamer must be located inside:
  ```
  manual_mode/mjpg-streamer/mjpg-streamer-experimental/
  ```
- The USB camera is expected at `/dev/video0`. You can change this in `scripts/start_streamer.sh`.

---

## 📁 Directory Structure

```
manual_mode/
├── launch/
│   └── manual_mode.launch
├── scripts/
│   └── start_streamer.sh
├── mjpg-streamer/
│   └── mjpg-streamer-experimental/
├── CMakeLists.txt
└── package.xml
```

---

## 🧪 Tested On

- Ubuntu 20.04
- ROS Noetic
- MJPG-Streamer (UVCCapture plugin)

---
