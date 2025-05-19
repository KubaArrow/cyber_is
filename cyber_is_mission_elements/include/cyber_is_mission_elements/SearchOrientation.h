//
// Created by victoria on 15.05.25.
//

#ifndef SEARCHORIENTATION_H
#define SEARCHORIENTATION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>




class SearchOrientation   {
public:
    explicit SearchOrientation(ros::NodeHandle &nh,
    const std::string &state_topic,
    const std::string &odom_topic,
    const std::string &line_detector_topic,
    const std::string &wall_topic);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    ros::Publisher state_pub_, wall_pub_;
    ros::Subscriber line_sub_, odom_sub_;

    std::string state_topic_, line_detector_topic_, wall_topic_, odom_topic_, wall_start_;

    double move_front_ = 1.5; // [m]
    double move_side_ = 1.0; // [m]
    double cage_height_ = 1.0; // [m]
    double cage_width_ = 1.0; // [m]
    double cage_size_ = 1.0; // [m]
    double cage_move_ = 1.0; // [m]
    bool turn_left_ = true;

    geometry_msgs::Point cornerPoint;
    geometry_msgs::Pose current_pose_;

    bool search_active_ = false;
    bool full_line_detected_ = false;
    bool founded_orientation_ = false;


    void executeSequence();

    void sendRelativeGoal(double dx, double dy, double dyaw);

    bool waitForResult() const;
    bool waitForResultWithAbordOnDone() const;

    void make_walls();

    void lineCallback(const std_msgs::String::ConstPtr &msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void publishAbort() const;

    void publishState(const std::string &msg) const;

};


#endif //SEARCHORIENTATION_H
