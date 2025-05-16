//
// Created by victoria on 11.04.25.
//
#include "cyber_is_filters/line_filter.h"

LineFilter::LineFilter(ros::NodeHandle& nh, const std::string& topic_name)
    : nh_(nh), input_topic_(topic_name)
{
    nh_.param("/line_filter/threshold0", treshhold[0], 100);
    nh_.param("/line_filter/threshold1", treshhold[1], 100);
    nh_.param("/line_filter/threshold2", treshhold[2], 100);
    nh_.param("/line_filter/threshold3", treshhold[3], 100);
    nh_.param("/line_filter/threshold4", treshhold[4], 100);

    nh_.param("max_limit", max_limit_, false);

    sub_ = nh_.subscribe(input_topic_, 10, &LineFilter::callback, this);
    pub_ = nh_.advertise<std_msgs::ByteMultiArray>(input_topic_ + "_filtered", 10);
    pos_pub_ = nh_.advertise<std_msgs::String>(input_topic_ + "_position", 10);


    ROS_INFO("LineFilter subscribed to: %s, publishing to: %s, thresholds: %d, %d, %d, %d, %d",
             input_topic_.c_str(), (input_topic_ + "_filtered").c_str(), treshhold[0], treshhold[1], treshhold[2],treshhold[3], treshhold[4]);
}

void LineFilter::callback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{

    std_msgs::ByteMultiArray result;
    int index=0;
    for (auto val : msg->data)
    {
        if (val >= treshhold[index])
        {
            result.data.push_back(true);
        }else{
          result.data.push_back(false);
         }
        index++;
    }

    std_msgs::String status;
    std::vector<int8_t> signedData(result.data.begin(), result.data.end());
    status.data = interpretPosition(signedData);
    ROS_INFO_STREAM("Detected line position: " << status.data);
    pos_pub_.publish(status);
    pub_.publish(result);

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
            weighted_sum += i;  // im dalej w prawo, tym większy indeks
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

