#include <ros/ros.h>
#include "cyber_is_filters/line_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_filter_node");
    ros::NodeHandle nh("~");  // private namespace for params

    std::string topic_name;
    nh.param<std::string>("line_detector_topic", topic_name, "/line_detector");

    LineFilter filter(nh, topic_name);

    ros::spin();
    return 0;
}
