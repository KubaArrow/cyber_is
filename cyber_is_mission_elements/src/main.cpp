#include <ros/ros.h>

#include "cyber_is_mission_elements/MissionController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle nh;
    ROS_INFO("START MISSION CONTROLLER");
    MissionController mc(nh);
    ros::Rate rate(10);
    ros::spin();
    return 0;
}

//==============================  end of file  ==============================
