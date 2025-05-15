//
// Created by victoria on 15.05.25.
//

#include "cyber_is_mission_elements/MissionCollector.h"
#include <std_msgs/Float32.h>


MissionCollector::MissionCollector(ros::NodeHandle &nh): nh_(nh) {
    nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>("battery_topic", battery_topic_, "/battery_state");

    nh_.param<std::string>("duration_topic", duration_topic_, "/mission_duration");
    nh_.param<std::string>("length_topic", length_topic_, "/mission_length");
    nh_.param<std::string>("energy_topic", energy_topic_, "/mission_energy");

    nh_.param<double>("publish_rate", publish_rate_, 1.0);


    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MissionCollector::odomCallback, this);
    battery_sub_ = nh_.subscribe(battery_topic_, 10, &MissionCollector::batteryCallback, this);

    duration_pub_ = nh_.advertise<std_msgs::Float32>(duration_topic_, 1);
    length_pub_ = nh_.advertise<std_msgs::Float32>(length_topic_, 1);
    energy_pub_ = nh_.advertise<std_msgs::Float32>(energy_topic_, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &MissionCollector::timerCallback, this);
}

void MissionCollector::startCollecting() {
    collecting_ = true;
    start_time_ = ros::Time::now();
    mission_duration_ = 0.0;
    mission_length_ = 0.0;
    mission_energy_ = 0.0;
    first_odom_ = true;
    first_battery_ = true;
    ROS_INFO("START COLLECTING");

}

void MissionCollector::stopCollecting() {
    collecting_ = false;
    ROS_INFO("STOP COLLECTING");
}

void MissionCollector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!collecting_) return;

    const geometry_msgs::Point& pos = msg->pose.pose.position;
    if (first_odom_) {
        last_position_ = pos;
        first_odom_ = false;
        return;
    }

    const double dx = pos.x - last_position_.x;
    const double dy = pos.y - last_position_.y;
    const double dz = pos.z - last_position_.z;

    mission_length_ += std::sqrt(dx*dx + dy*dy + dz*dz);
    last_position_ = pos;
}

void MissionCollector::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    if (!collecting_) return;

    last_charge_ = msg->charge;

    if (first_battery_) {
        start_charge_ = msg->charge;
        first_battery_ = false;
    }
}


void MissionCollector::timerCallback(const ros::TimerEvent&) {
    if (!collecting_) return;

    mission_duration_ = (ros::Time::now() - start_time_).toSec();

    std_msgs::Float32 duration_msg;
    duration_msg.data = mission_duration_;
    duration_pub_.publish(duration_msg);

    std_msgs::Float32 length_msg;
    length_msg.data = mission_length_;
    length_pub_.publish(length_msg);

    std_msgs::Float32 energy_msg;
    energy_msg.data = std::max(0.0, start_charge_ - last_charge_);  // unikamy ujemnych przez pomyłki
    energy_pub_.publish(energy_msg);
}