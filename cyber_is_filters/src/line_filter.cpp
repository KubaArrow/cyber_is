//
// Created by victoria on 11.04.25.
//
#include "cyber_is_filters/line_filter.h"

LineFilter::LineFilter()
  : rclcpp::Node("line_filter")
{
    input_topic_ = this->declare_parameter<std::string>("line_detector_topic", "/line_detector");
    treshhold[0] = this->declare_parameter<int>("threshold0", 100);
    treshhold[1] = this->declare_parameter<int>("threshold1", 100);
    treshhold[2] = this->declare_parameter<int>("threshold2", 100);
    treshhold[3] = this->declare_parameter<int>("threshold3", 100);
    treshhold[4] = this->declare_parameter<int>("threshold4", 100);

    max_limit_ = this->declare_parameter<bool>("max_limit", false);

    sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        input_topic_, 10,
        std::bind(&LineFilter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(input_topic_ + "_filtered", 10);
    pos_pub_ = this->create_publisher<std_msgs::msg::String>(input_topic_ + "_position", 10);

    RCLCPP_INFO(this->get_logger(),
                "LineFilter subscribed to: %s, publishing to: %s, thresholds: %d, %d, %d, %d, %d",
                input_topic_.c_str(), (input_topic_ + "_filtered").c_str(), treshhold[0], treshhold[1], treshhold[2], treshhold[3], treshhold[4]);
}

void LineFilter::callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    std_msgs::msg::ByteMultiArray result;
    int index = 0;
    for (auto val : msg->data)
    {
        if (val >= treshhold[index])
        {
            result.data.push_back(1);
        }
        else
        {
            result.data.push_back(0);
        }
        index++;
    }

    std_msgs::msg::String status;
    std::vector<int8_t> signedData(result.data.begin(), result.data.end());
    status.data = interpretPosition(signedData);
    pos_pub_->publish(status);
    pub_->publish(result);
}

std::string LineFilter::interpretPosition(const std::vector<int8_t>& data)
{
    if (data == std::vector<int8_t>{0,0,0,0,0}) return "NO_LINE";
    if (data == std::vector<int8_t>{1,1,1,1,1}) return "FULL_LINE";

    int count = 0;
    int weighted_sum = 0;

    for (size_t i = 0; i < data.size(); ++i)
    {
        if (data[i])
        {
            ++count;
            weighted_sum += static_cast<int>(i);  // im dalej w prawo, tym większy indeks
        }
    }

    if (count == 0)
        return "NO_LINE";

    double average_pos = static_cast<double>(weighted_sum) / count;

    if (count >= 4)
        return "WIDE_LINE";
    else if (average_pos < 2.0)
        return "LEFT";
    else if (average_pos < 2.5)
        return "CENTER";
    else
        return "RIGHT";
}
