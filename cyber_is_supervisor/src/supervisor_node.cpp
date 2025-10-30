// ROS 2 Humble rclcpp supervisor node
// - Listens on `/robot_mode` (std_msgs/String): "MANUAL" or "AUTO"
// - Publishes `/robot_state` (std_msgs/String) with transient local QoS (latch-like)
// - Publishes `/leds_mode` (std_msgs/String)
// - Optional echo test: `/supervisor/echo_in` -> `/supervisor/echo_out`
// - Heartbeat timer on `/supervisor/heartbeat`
// - Autonomy mode is a placeholder (no process launching)

#include <chrono>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class SupervisorNode : public rclcpp::Node {
public:
  SupervisorNode()
  : Node("supervisor_node")
  {
    // Parameters
    this->declare_parameter<std::string>("initial_mode", "MANUAL");
    this->declare_parameter<std::string>("nav_type", "n");
    this->declare_parameter<bool>("enable_echo", true);
    this->declare_parameter<double>("heartbeat_period", 1.0);

    // QoS for latched-like behavior on robot_state
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();
    latched_qos.reliable();

    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_state", latched_qos);
    leds_pub_        = this->create_publisher<std_msgs::msg::String>("/leds_mode", 10);
    cmd_pub_         = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    heartbeat_pub_   = this->create_publisher<std_msgs::msg::String>("/supervisor/heartbeat", 10);

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_mode", 10,
      std::bind(&SupervisorNode::onMode, this, std::placeholders::_1));

    bool enable_echo = this->get_parameter("enable_echo").as_bool();
    if (enable_echo) {
      echo_pub_ = this->create_publisher<std_msgs::msg::String>("/supervisor/echo_out", 10);
      echo_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/supervisor/echo_in", 10,
        std::bind(&SupervisorNode::onEcho, this, std::placeholders::_1));
    }

    const double hb = this->get_parameter("heartbeat_period").as_double();
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(hb)),
      std::bind(&SupervisorNode::onHeartbeat, this));

    // Initialize mode
    current_mode_ = this->get_parameter("initial_mode").as_string();
    toUpper(current_mode_);
    if (current_mode_ != "AUTO" && current_mode_ != "MANUAL") {
      RCLCPP_WARN(this->get_logger(), "Invalid initial_mode '%s', defaulting to MANUAL", current_mode_.c_str());
      current_mode_ = "MANUAL";
    }
    if (current_mode_ == "MANUAL") {
      enterManual();
    } else {
      enterAuto();
    }

    RCLCPP_INFO(this->get_logger(), "Supervisor ready. Listening on /robot_mode ...");
  }

private:
  void onMode(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string m = msg->data;
    toUpper(m);
    if (m == current_mode_) {
      // no-op
      return;
    }
    if (m == "MANUAL") {
      enterManual();
    } else if (m == "AUTO") {
      enterAuto();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown mode received: '%s'", m.c_str());
    }
  }

  void enterManual()
  {
    current_mode_ = "MANUAL";
    RCLCPP_INFO(this->get_logger(), "Switching to MANUAL mode");
    publishLed("FRONT_WHITE_25");
    publishLed("SIDE_GREEN_BREATH");
    publishState("START_MANUAL_MODE");
    publishLed("FRONT_WHITE_100");
    publishLed("SIDE_GREEN_75");
    publishState("READY_MANUAL_MODE");
  }

  void enterAuto()
  {
    current_mode_ = "AUTO";
    RCLCPP_INFO(this->get_logger(), "Switching to AUTO mode (placeholder)");
    publishState("START_AUTONOMY_MODE");
    publishLed("FRONT_RED_BREATH");
    publishLed("SIDE_RED_LOAD");
    publishLed("SIDE_RED_25");
    publishState("READY_AUTONOMY_MODE");
  }

  void onEcho(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!echo_pub_) return;
    echo_pub_->publish(*msg);
  }

  void onHeartbeat()
  {
    std_msgs::msg::String hb;
    hb.data = std::string("OK ") + current_mode_;
    heartbeat_pub_->publish(hb);
  }

  void publishState(const std::string & s)
  {
    std_msgs::msg::String m; m.data = s; robot_state_pub_->publish(m);
  }

  void publishLed(const std::string & s)
  {
    std_msgs::msg::String m; m.data = s; leds_pub_->publish(m);
  }

  static void toUpper(std::string & s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::toupper(c); });
  }

private:
  std::string current_mode_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leds_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr echo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr echo_sub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

