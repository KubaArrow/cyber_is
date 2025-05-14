#include <ros/ros.h>
#include "cyber_is_filters/magnet_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magnet_filter_node");
    ros::NodeHandle nh("~");  // private namespace for params

    std::string topic_name;
    nh.param<std::string>("magnet_detector_topic", topic_name, "/magnet");

    MagnetFilter filter(nh, topic_name);

    ros::spin();
    return 0;
}
