#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <std_msgs/String.h> 

class MagnetFilter
{
public:
    MagnetFilter(ros::NodeHandle& nh, const std::string& topic_name);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher robot_state_pub_;
    std::string input_topic_;
    bool max_limit_;
    int min_;
    int max_;

    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
};
