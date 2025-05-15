//
// Created by victoria on 15.05.25.
//

#ifndef MISSIONCONTROLLER_H
#define MISSIONCONTROLLER_H

#include <string>
#include <variant>
#include <cyber_is_mission_elements/MissionCollector.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cyber_is_mission_elements/SearchMeta.h"
#include "cyber_is_mission_elements/SearchOrientation.h"
#include "cyber_is_mission_elements/SearchStart.h"
#include "cyber_is_mission_elements/ZoneNavigator.h"



class MissionController {
public:
    explicit MissionController(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber state_sub_;
    MissionCollector m_coll_;


    using ElementVariant = std::variant<
        std::monostate,
        SearchStart,
        SearchOrientation,
        ZoneNavigator,
        SearchMeta
    >;
    ElementVariant mission_element_{std::monostate{}};


    std::string state_topic_ = "/robot_state";
    std::string odom_topic_ = "/odometry/filtered";
    std::string line_detector_topic_="/line_detector_position";
    std::string wall_topic_="/virtual_obstacles";

    void stateCallback(const std_msgs::String::ConstPtr &msg);

    void publishState(const std::string &msg) const;
};


#endif //MISSIONCONTROLLER_H
