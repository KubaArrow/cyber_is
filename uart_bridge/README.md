
# uart_bridge

`uart_bridge` is a ROS package that facilitates bidirectional communication between a ROS-based system and a low-level microcontroller using the SLIP (Serial Line Internet Protocol) protocol. It supports the transfer of control commands and sensor data, and translates between ROS messages and a custom binary protocol.

---

## Features

- SLIP encoding/decoding and CRC8 checksum verification.
- Threaded serial communication with automatic reconnection.
- Publishers for:
  - IMU (`sensor_msgs/Imu`)
  - Odometry (`nav_msgs/Odometry`)
  - Battery state (`sensor_msgs/BatteryState`)
  - Magnetic field (`std_msgs/Float64MultiArray`)
  - Line detector (`std_msgs/UInt16MultiArray`)
  - Diagnostic status (`diagnostic_msgs/DiagnosticStatus`)
- Subscribers for:
  - Velocity command (`geometry_msgs/Twist`)
  - LED control (`std_msgs/UInt8MultiArray`)

---

## Dependencies

- ROS (tested with ROS Noetic)
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `diagnostic_msgs`
- `std_msgs`

---

## Build

Clone the repository into your catkin workspace and build:

```bash
cd ~/catkin_ws/src
git clone <your-repo-url>
cd ..
catkin_make
source devel/setup.bash
```

---

## Run

You can launch the bridge using the provided launch file:

```bash
roslaunch uart_bridge start_bridge.launch
```

Default parameters can be overridden via launch arguments:

```xml
<param name="uart_port" value="/dev/ttyACM0"/>
<param name="twist_topic" value="/cmd_vel" />
<param name="odom_topic" value="/low_level_odom" />
<param name="imu_topic" value="/imu" />
<param name="magnet_topic" value="/magnet" />
<param name="line_detector_topic" value="/line_detector" />
<param name="leds_topic" value="/leds"/>
<param name="battery_topic" value="/battery"/>
<param name="status_topic" value="/status"/>
```

---

## Serial Protocol

The microcontroller is expected to send and receive SLIP-encoded packets with the following message types:

| Message ID       | Description         | ROS Type                         |
|------------------|---------------------|----------------------------------|
| `0x30`           | IMU                 | `sensor_msgs/Imu`                |
| `0x31`           | Line Detector       | `std_msgs/UInt16MultiArray`      |
| `0x32`           | Velocity Command    | `geometry_msgs/Twist`            |
| `0x33`           | Battery State       | `sensor_msgs/BatteryState`       |
| `0x34`           | Magnetometer        | `std_msgs/Float64MultiArray`     |
| `0x35`           | Odometry            | `nav_msgs/Odometry`              |
| `0x36`           | LED Colors          | `std_msgs/UInt8MultiArray`       |
| `0x37`           | Diagnostics         | `diagnostic_msgs/DiagnosticStatus` |

All messages include a CRC8 byte for integrity verification.

---

## Logging

The bridge uses prefixed logging macros (`SS_LOG_DEBUG`, `SS_LOG_ERROR`) for internal diagnostics. Logs are output to `stderr` and the ROS console.

---

## Error Handling

The bridge implements:
- Automatic serial reconnection if the port is disconnected.
- CRC checking on all received messages.
- SLIP buffer overflow prevention and error reporting.
- Diagnostics publishing (`diagnostic_msgs/DiagnosticStatus`).

---

## Customization

Message structures and SLIP decoding behavior can be customized in `rosTopic.h` and `serial_slip.c` respectively.

---

## License

MIT License (or specify your actual license)

---

## Author

Created by `bkfcb` and `kuba_Arrow`, March 2025

