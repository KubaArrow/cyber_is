#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/String.h>

class LineFilter
{
public:
    LineFilter(ros::NodeHandle& nh, const std::string& topic_name);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pos_pub_;
    std::string input_topic_;
    bool max_limit_;
    int min_;
    int max_;

    void callback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
    std::string interpretPosition(const std::vector<int8_t>& data);
};
