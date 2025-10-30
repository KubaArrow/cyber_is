// ROS2 / rclcpp port

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

#include "uart_bridge/serial_slip.h"
#include "uart_bridge/rosTopic.h"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class UartBridgeNode : public rclcpp::Node {
public:
  UartBridgeNode() : rclcpp::Node("uart_bridge"), slip_(nullptr), counter_(0) {
    // Declare parameters
    uart_port_ = this->declare_parameter<std::string>("uart_port", "/dev/ttyACM0");
    frequency_ = this->declare_parameter<int>("frequency", 100);

    twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/cmd_vel");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/slam_out_pose");

    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/low_level_odom");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "map");

    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu");

    magnet_topic_ = this->declare_parameter<std::string>("magnet_topic", "/magnet");
    line_detector_topic_ = this->declare_parameter<std::string>("line_detector_topic", "/line_detector");
    leds_topic_ = this->declare_parameter<std::string>("leds_topic", "/leds");
    battery_topic_ = this->declare_parameter<std::string>("battery_topic", "/battery");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/status");

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(10));
    magnet_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(magnet_topic_, rclcpp::QoS(10));
    line_detector_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(line_detector_topic_, rclcpp::QoS(10));
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, rclcpp::QoS(10));
    status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(status_topic_, rclcpp::QoS(10));

    // Subscribers
    using std::placeholders::_1;
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic_, rclcpp::QoS(10), std::bind(&UartBridgeNode::cmdVelCallback, this, _1));
    sub_leds_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        leds_topic_, rclcpp::QoS(10), std::bind(&UartBridgeNode::ledsCallback, this, _1));
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::QoS(10), std::bind(&UartBridgeNode::poseCallback, this, _1));

    // Initial status
    this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "ROS STARTED",
                               "Ros2 read params, prepared subscribers and publishers");

    // Open serial
    slip_ = serial_slip_open(uart_port_.c_str());
    if (!slip_) {
      this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "FAILED SERIAL OPENED",
                                 "Failed to open serial slip");
    } else {
      this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "SERIAL OPENED",
                                 "Serial slip opened");
    }

    // Timer for polling serial and periodic diagnostics
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1, frequency_)));
    last_diag_time_ = this->now();
    this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "READY", "Start ROS2 loop");
    timer_ = this->create_wall_timer(period, std::bind(&UartBridgeNode::timerCallback, this));
  }

  ~UartBridgeNode() override {
    if (slip_) {
      serial_slip_close(slip_);
      slip_ = nullptr;
    }
  }

private:
  void publishPackageStatus(uint8_t level, const std::string &name, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;  // 0 = OK, 1 = WARN, 2 = ERROR
    status.name = name;
    status.message = message;
    status.hardware_id = "uart_bridge";

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Timestamp";
    kv.value = std::to_string(this->now().seconds());
    status.values.push_back(kv);

    status_pub_->publish(status);
  }

  bool ensureSlip() {
    if (slip_) return true;
    slip_ = serial_slip_open(uart_port_.c_str());
    if (!slip_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial slip on %s", uart_port_.c_str());
      return false;
    }
    this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "SERIAL OPENED", "Serial slip opened");
    return true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    CmdVelMsg_t packet{};
    packet.msgID = CMD_VEL_MSG;
    packet.linear.x = msg->linear.x;
    packet.linear.y = msg->linear.y;
    packet.linear.z = msg->linear.z;
    packet.angular.x = msg->angular.x;
    packet.angular.y = msg->angular.y;
    packet.angular.z = msg->angular.z;

    if (!ensureSlip()) return;
    auto result = serial_slip_write(slip_, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result != SS_OK) {
      RCLCPP_ERROR(this->get_logger(), "serial_slip_write failed: %s", serial_slip_decode_error(result));
    }
  }

  void ledsCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    LEDMsg_t packet{};
    packet.msgID = LED_MSG;

    const size_t leds = 23;
    if (msg->data.size() < leds * 4) {
      RCLCPP_WARN(this->get_logger(), "LED array too small: %zu < %zu", msg->data.size(), leds * 4);
    }
    for (size_t i = 0; i < leds; i++) {
      size_t base = i * 4;
      if (base + 3 < msg->data.size()) {
        packet.leds[i].R = msg->data[base];
        packet.leds[i].G = msg->data[base + 1];
        packet.leds[i].B = msg->data[base + 2];
        packet.leds[i].A = msg->data[base + 3];
      }
    }
    if (!ensureSlip()) return;
    auto result = serial_slip_write(slip_, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result != SS_OK) {
      RCLCPP_ERROR(this->get_logger(), "serial_slip_write failed: %s", serial_slip_decode_error(result));
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    PoseMsg_t packet{};
    packet.msgID = POSE_MSG;
    packet.x = msg->pose.position.x;
    packet.y = msg->pose.position.y;
    const auto &q = msg->pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    packet.yaw = std::atan2(siny_cosp, cosy_cosp);

    if (!ensureSlip()) return;
    auto result = serial_slip_write(slip_, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result != SS_OK) {
      RCLCPP_ERROR(this->get_logger(), "serial_slip_write failed: %s", serial_slip_decode_error(result));
    }
  }

  void timerCallback() {
    // Try to open if not yet opened
    if (!slip_) {
      (void)ensureSlip();
    }

    // Poll non-blocking for a single message
    if (slip_) {
      uint8_t recvBuffer[8192];
      size_t recvLen = 0;
      auto err = serial_slip_get_message(slip_, recvBuffer, sizeof(recvBuffer), &recvLen, 0);
      if (err == SS_OK) {
        switch ((msgID_t)recvBuffer[0]) {
          case IMU_MSG: {
            auto *imuMsg = (ImuMsg_t *)recvBuffer;
            sensor_msgs::msg::Imu ros_imu;
            ros_imu.header.stamp = this->now();
            ros_imu.header.frame_id = imu_frame_;
            ros_imu.orientation.x = imuMsg->orientation.x;
            ros_imu.orientation.y = imuMsg->orientation.y;
            ros_imu.orientation.z = imuMsg->orientation.z;
            ros_imu.orientation.w = imuMsg->orientation.w;
            for (int i = 0; i < 9; i++) {
              ros_imu.orientation_covariance[i] = 0.0;
              ros_imu.angular_velocity_covariance[i] = 0.0;
              ros_imu.linear_acceleration_covariance[i] = 0.0;
            }
            ros_imu.angular_velocity.x = imuMsg->angular_velocity.x;
            ros_imu.angular_velocity.y = imuMsg->angular_velocity.y;
            ros_imu.angular_velocity.z = imuMsg->angular_velocity.z;
            ros_imu.linear_acceleration.x = imuMsg->linear_acceleration.x;
            ros_imu.linear_acceleration.y = imuMsg->linear_acceleration.y;
            ros_imu.linear_acceleration.z = imuMsg->linear_acceleration.z;
            imu_pub_->publish(ros_imu);
            break;
          }
          case TRACKER_SENSOR_MSG: {
            auto *trackerMsg = (TrackerSensorMsg_t *)recvBuffer;
            std_msgs::msg::UInt16MultiArray ros_msg;
            for (int i = 0; i < 5; ++i) {
              ros_msg.data.push_back(trackerMsg->data[i]);
            }
            ros_msg.layout.dim.resize(1);
            ros_msg.layout.dim[0].label = "lines";
            ros_msg.layout.dim[0].size = 5;
            ros_msg.layout.dim[0].stride = 5;
            ros_msg.layout.data_offset = 0;
            line_detector_pub_->publish(ros_msg);
            break;
          }
          case BATTERY_STATE_MSG: {
            auto *batteryMsg = (BatteryStateMsg_t *)recvBuffer;
            sensor_msgs::msg::BatteryState battery;
            battery.header.stamp = this->now();
            battery.voltage = batteryMsg->voltage;
            battery.current = batteryMsg->current;
            battery.charge = static_cast<float>(batteryMsg->energy);
            battery.percentage = batteryMsg->percentage / 100.0f;
            battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
            battery.present = true;
            battery_pub_->publish(battery);
            break;
          }
          case MAGNET_MSG: {
            auto *magnetMsg = (MagnetMsg_t *)recvBuffer;
            std_msgs::msg::Float64MultiArray magnet_array;
            magnet_array.data.push_back(magnetMsg->magnet.x);
            magnet_array.data.push_back(magnetMsg->magnet.y);
            magnet_array.data.push_back(magnetMsg->magnet.z);
            magnet_pub_->publish(magnet_array);
            break;
          }
          case ODOMETRY_MSG: {
            auto *odomMsg = (OdometryMsg_t *)recvBuffer;
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->now();
            odom.header.frame_id = odom_frame_;
            odom.pose.pose.position.x = odomMsg->position.x;
            odom.pose.pose.position.y = odomMsg->position.y;
            odom.pose.pose.position.z = odomMsg->position.z;
            odom.pose.pose.orientation.x = odomMsg->orientation.x;
            odom.pose.pose.orientation.y = odomMsg->orientation.y;
            odom.pose.pose.orientation.z = odomMsg->orientation.z;
            odom.pose.pose.orientation.w = odomMsg->orientation.w;
            odom.twist.twist.linear.x = odomMsg->linear.x;
            odom.twist.twist.linear.y = odomMsg->linear.y;
            odom.twist.twist.linear.z = odomMsg->linear.z;
            odom.twist.twist.angular.x = odomMsg->angular.x;
            odom.twist.twist.angular.y = odomMsg->angular.y;
            odom.twist.twist.angular.z = odomMsg->angular.z;
            odom_pub_->publish(odom);
            break;
          }
          case DIAG_MSG: {
            auto *diagMsg = (DiagMsg_t *)recvBuffer;
            std::string name(reinterpret_cast<const char *>(diagMsg->name), diagMsg->nameLen);
            std::string message(reinterpret_cast<const char *>(diagMsg->msg), diagMsg->msgLen);
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.level = diagMsg->level;  // 0 = OK, 1 = WARN, 2 = ERROR
            status.name = name;
            status.message = message;
            status.hardware_id = "low_level_board";
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "Timestamp";
            kv.value = std::to_string(this->now().seconds());
            status.values.push_back(kv);
            status_pub_->publish(status);
            break;
          }
          default:
            break;
        }
      }
    }

    // Periodic heartbeat every second
    if ((this->now() - last_diag_time_).seconds() >= 1.0) {
      counter_++;
      this->publishPackageStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "PACKAGE",
                                 std::string("WORK: ") + std::to_string(counter_));
      last_diag_time_ = this->now();
    }
  }

  // Members
  SerialSlip *slip_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Pubs / Subs
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr magnet_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr line_detector_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_leds_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;

  // Params
  std::string uart_port_;
  int frequency_;
  std::string twist_topic_, pose_topic_;
  std::string odom_topic_, imu_topic_, magnet_topic_, line_detector_topic_, leds_topic_, battery_topic_, status_topic_;
  std::string odom_frame_, imu_frame_;

  // State
  rclcpp::Time last_diag_time_;
  uint64_t counter_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
