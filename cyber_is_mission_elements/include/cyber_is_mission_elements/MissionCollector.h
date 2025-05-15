//
// Created by victoria on 15.05.25.
//

#ifndef MISSIONCOLLECTOR_H
#define MISSIONCOLLECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>


class MissionCollector {

public:
    explicit MissionCollector(ros::NodeHandle &nh);
    void startCollecting();
    void stopCollecting();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, battery_sub_;
    ros::Publisher duration_pub_, length_pub_, energy_pub_;
    ros::Timer timer_;

    std::string odom_topic_, battery_topic_;
    std::string duration_topic_, length_topic_, energy_topic_;
    double publish_rate_;

    bool collecting_ = false;
    ros::Time start_time_;
    double mission_duration_ = 0.0;
    double mission_length_ = 0.0;
    double mission_energy_ = 0.0;
    double start_charge_ = 0.0;
    double last_charge_ = 0.0;
    bool first_battery_ = true;


    geometry_msgs::Point last_position_;
    bool first_odom_ = true;


    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);

};



#endif //MISSIONCOLLECTOR_H
