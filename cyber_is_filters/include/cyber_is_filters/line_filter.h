
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class LineFilter : public rclcpp::Node
{
public:
    LineFilter();

private:
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pos_pub_;
    std::string input_topic_;
    bool max_limit_{};
    int treshhold[5] = {100,100,100,100,100};

    void callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    std::string interpretPosition(const std::vector<int8_t>& data);
};
