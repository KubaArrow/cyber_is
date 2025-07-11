# Project Cyber IS
Package to bringup robot


### 1. **Connect with Raspberry**

```bash
ssh pi@192.168.18.101
```

```bash
cd catkin_ws/src
```

```bash
git clone https://github.com/KubaArrow/cyber_is_bringup.git
```

```bash
cd .. && catkin_make
```

```bash
sudo chmod +x /home/pi/catkin_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh
```

### 2. **Create service systemd**


```bash
sudo nano /etc/systemd/system/roscore.service
```
```ini
[Unit]
Description=ROS Core
After=network.target
Requires=network.target

[Service]
ExecStart=/opt/ros/noetic/bin/roscore
Restart=on-failure
User=pi
Environment=DISPLAY=:0
Environment=ROS_IP=192.168.1.11
Environment=ROS_MASTER_URI=http://192.168.1.11:11311

[Install]
WantedBy=multi-user.target
```

```bash
sudo nano /etc/systemd/system/ros_start.service
```

```ini
[Unit]
Description=Start ROS Packages After roscore
After=roscore.service
Requires=roscore.service

[Service]
ExecStart=/home/pi/catkin_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh
Restart=on-failure
User=pi
Environment=DISPLAY=:0
Environment=ROS_IP=192.168.1.11
Environment=ROS_MASTER_URI=http://192.168.1.11:11311

[Install]
WantedBy=multi-user.target
```

### 3. **Start service**

```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

sudo systemctl enable roscore.service
sudo systemctl enable ros_start.service

sudo systemctl status roscore.service
sudo systemctl status ros_start.service

sudo systemctl restart roscore.service
sudo systemctl restart ros_start.service

sudo systemctl start roscore.service

```

Możesz sprawdzić status:

```bash
sudo systemctl status ros_start.service
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 odom2 map 100
rosrun tf static_transform_publisher 0 0 0 0 0 0.9999997 0.0007963 odom2 map 100
rosrun tf static_transform_publisher 0 0 0 0 0 0.9999997 0.0007963 odom odom_rotated 100

```
```bash
 cd mjpg-streamer/mjpg-streamer-experimental/
./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "./output_http.so -p 8080 -w ./www"

sudo systemctl restart ros_start.service

```