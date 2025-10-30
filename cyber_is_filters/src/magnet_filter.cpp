//
// Created by victoria on 11.04.25.
//
#include "cyber_is_filters/magnet_filter.h"

MagnetFilter::MagnetFilter()
  : rclcpp::Node("magnet_filter")
{
    input_topic_ = this->declare_parameter<std::string>("magnet_detector_topic", "/magnet");
    min_ = this->declare_parameter<int>("min_value", 100);
    max_ = this->declare_parameter<int>("max_value", 10);
    max_limit_ = this->declare_parameter<bool>("max_limit", false);

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        input_topic_, 10,
        std::bind(&MagnetFilter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Bool>(input_topic_ + "_filtered", 10);
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_state", 10);

    RCLCPP_INFO(this->get_logger(),
                "MagnetFilter subscribed to: %s, publishing to: %s, min: %d, max: %d",
                input_topic_.c_str(), (input_topic_ + "_filtered").c_str(), min_, max_);
}

void MagnetFilter::callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std_msgs::msg::Bool result;
    if (msg->data.size() >= 3)
    {
        double val = std::sqrt((msg->data[0]*msg->data[0]) +(msg->data[1]*msg->data[1]) + (msg->data[2]*msg->data[2]));
        if (val >= min_ && (val <= max_ || !max_limit_))
        {
            result.data = true;
            std_msgs::msg::String state_msg;
            state_msg.data = "FOUNDED_FINISH";
            robot_state_pub_->publish(state_msg);
        }
        else
        {
            result.data = false;
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "MagnetFilter: message has fewer than 3 elements!");
        result.data = false;  // domyślna wartość, jeśli dane są niepoprawne
    }

    pub_->publish(result);
}


