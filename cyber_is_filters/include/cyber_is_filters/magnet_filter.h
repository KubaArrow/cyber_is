#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class MagnetFilter : public rclcpp::Node
{
public:
    MagnetFilter();

private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
    std::string input_topic_;
    bool max_limit_{};
    int min_{};
    int max_{};

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};
