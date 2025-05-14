//
// Created by victoria on 11.04.25.
//
#include "cyber_is_filters/magnet_filter.h"

MagnetFilter::MagnetFilter(ros::NodeHandle& nh, const std::string& topic_name)
    : nh_(nh), input_topic_(topic_name)
{
    nh_.param("min_value", min_, 100);
    nh_.param("max_value", max_, 10);
    nh_.param("max_limit", max_limit_, false);

    sub_ = nh_.subscribe(input_topic_, 10, &MagnetFilter::callback, this);
    pub_ = nh_.advertise<std_msgs::Bool>(input_topic_ + "_filtered", 10);

    ROS_INFO("LineFilter subscribed to: %s, publishing to: %s, min: %d, max: %d",
             input_topic_.c_str(), (input_topic_ + "_filtered").c_str(), min_, max_);
}

void MagnetFilter::callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std_msgs::Bool result;
    ROS_INFO("MagnetFilter: received message with %lu elements", msg->data.size());
    // Upewnij się, że wiadomość zawiera co najmniej 3 elementy
    if (msg->data.size() >= 3)
    {
        double val1 = abs(msg->data[0]);
        double val2 = abs(msg->data[1]);
        double val3 = abs(msg->data[2]);

        double val = 0;

        if (val1 > val2 && val1 > val3){
            val=val1;
        }else if (val2 > val1 && val2 > val3){
            val=val2;
        }else{
            val=val3;
        }
        if (val >= min_ && (val <= max_ || !max_limit_))
        {
            result.data = true;
        }
        else
        {
            result.data = false;
        }
    }
    else
    {
        ROS_WARN("MagnetFilter: message has fewer than 3 elements!");
        result.data = false;  // domyślna wartość, jeśli dane są niepoprawne
    }

    pub_.publish(result);
}



