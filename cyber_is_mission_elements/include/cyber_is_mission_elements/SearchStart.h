//
// Created by victoria on 15.05.25.
//

#ifndef SEARCHSTART_H
#define SEARCHSTART_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>



class SearchStart {
public:
    SearchStart(ros::NodeHandle& nh,
           const std::string& state_topic,
           const std::string& odom_topic,
           const std::string& line_detector_topic,
           const std::string& wall_topic);
private:
    ros::NodeHandle nh_;
    ros::Subscriber line_sub_, odom_sub_;
    ros::Publisher state_pub_, wall_pub_;
    geometry_msgs::Pose current_pose_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    std::string state_topic_, odom_topic_, line_detector_topic_, wall_topic_ ;


    bool search_active_ = false;
    bool full_line_detected_ = false;

    void executeSequence();
    void sendRelativeGoal(double dx, double dy, double dyaw);
    bool waitForResult() const;
    void lineCallback(const std_msgs::String::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void make_wall() const;

    void publishAbort() const;
    void publishState(const std::string &msg) const;

};



#endif //SEARCHSTART_H
