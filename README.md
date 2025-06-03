# PROJECT CYBER IS

## Config static IP
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

```yaml
network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                SmithIndustries:
                    password: sztukadlasztuki
            dhcp4: no
            addresses:
              - 192.168.1.11/24
            gateway4: 192.168.1.1
            nameservers:
              addresses:
                - 8.8.8.8
                - 1.1.1.1
```
```bash
sudo netplan apply
```

## Installation ROS Noetic
[Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## Installation extras packages
```bash
sudo apt update && sudo apt install -y \
    ros-noetic-hector-slam \
    ros-noetic-rosbridge-suite \
    ros-noetic-navigation \
    ros-noetic-move-base \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-dwa-local-planner \
    ros-noetic-costmap-2d \
    ros-noetic-global-planner \
    ros-noetic-navfn \
    ros-noetic-robot-localization \
    ros-noetic-laser-filters \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-smach \
    ros-noetic-smach-ros
```

## Installation CYBER IS

```bash
cd catkin_ws/src
git clone https://github.com/KubaArrow/cyber_is.git
git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros.git
cd .. && catkin_make
sudo chmod +x /home/pi/catkin_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh

```

## Services

### Create
```bash
sudo nano /etc/systemd/system/roscore.service
```
```ini
[Unit]
Description=ROS Core
After=network.target
Requires=network.target

[Service]
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && exec roscore'
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

### Start

```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

sudo systemctl enable roscore.service
sudo systemctl enable ros_start.service

sudo systemctl start roscore.service
sudo systemctl start ros_start.service


```

### Restart

```bash
sudo systemctl restart roscore.service
sudo systemctl restart ros_start.service
```

### Status
```bash
sudo systemctl status roscore.service
sudo systemctl status ros_start.service
```

```bash
sudo journalctl -u roscore.service -f
sudo journalctl -u ros_start.service -f
```


## Testing

```bash
cat /sys/class/thermal/thermal_zone0/temp

```