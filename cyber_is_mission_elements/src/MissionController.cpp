//
// Created by victoria on 15.05.25.
//

#include "cyber_is_mission_elements/MissionController.h"
#include <std_msgs/String.h>

#include "cyber_is_mission_elements/SearchMeta.h"
#include "cyber_is_mission_elements/SearchOrientation.h"
#include "cyber_is_mission_elements/SearchStart.h"
#include "cyber_is_mission_elements/ZoneNavigator.h"

MissionController::MissionController(ros::NodeHandle &nh): nh_(nh), m_coll_(nh_)  {


    nh_.param<std::string>("/mission/state_topic", state_topic_, "/robot_state");
    nh_.param<std::string>("/mission/line_detector_topic", line_detector_topic_, "/line_detector_position");
    nh_.param<std::string>("/mission/odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>("/mission/wall_topic", wall_topic_, "/virtual_obstacles");

    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1);
    state_sub_ = nh_.subscribe(state_topic_, 1, &MissionController::stateCallback, this);

    ROS_INFO("[MissionController] Ready");
}

void MissionController::stateCallback(const std_msgs::String::ConstPtr &msg)  {

    if (msg->data == "START_MISSION") {
        m_coll_.startCollecting();
        mission_element_.emplace<SearchStart>(nh_, state_topic_, odom_topic_, line_detector_topic_, wall_topic_);
        ROS_INFO("START_MISSION received. Starting searching start pose.");
    }else if (msg->data == "FOUNDED_START_POSE") {
        mission_element_.emplace<SearchOrientation>(nh_, state_topic_, odom_topic_, line_detector_topic_, wall_topic_);
        ROS_INFO("FOUNDED_START_POSE received. Starting search orientation");

    }else if (msg->data == "FOUNDED_ORIENTATION") {
        mission_element_.emplace<ZoneNavigator>(nh_, state_topic_);

        ROS_INFO("FOUNDED_ORIENTATION received. Starting go to zone");
    }else if (msg->data == "FOUNDED_ZONE") {
        mission_element_.emplace<SearchMeta>(nh_, state_topic_, line_detector_topic_, odom_topic_ );
        ROS_INFO("FOUNDED_ZONE received. Starting search meta");
    }else if (msg->data == "FOUNDED_FINISH") {
        m_coll_.stopCollecting();
        ROS_INFO("FOUNDED_FINISH received. Stop collecting and start finish action");
    }else {
        ROS_INFO("Unrecognize status");
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

}

void MissionController::publishState(const std::string &msg) const {
    std_msgs::String s;
    s.data = msg;
    state_pub_.publish(s);
}
