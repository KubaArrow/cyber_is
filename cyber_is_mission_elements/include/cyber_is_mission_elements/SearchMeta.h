//
// Created by victoria on 15.05.25.
//

#ifndef SEARCHMETA_H
#define SEARCHMETA_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>



class SearchMeta   {
public:
    explicit SearchMeta(ros::NodeHandle &nh, const std::string &state_topic, const std::string &line_detector_topic, const std::string &odom_topic);

private:
    ros::NodeHandle nh_;
    using Point = std::pair<double, double>;
    std::vector<Point> points_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    std::string state_topic_, line_detector_topic_, magnet_topic_, odom_topic_;
    ros::Subscriber line_sub_, magnet_sub_, odom_sub_;
    ros::Publisher state_pub_;
    geometry_msgs::Pose current_pose_;

    bool search_active_ = false;
    bool full_line_detected_ = false;
    bool found_ = false;

    std::string last_line_status_ = "NO_LINE";
    std::string mode_ = "line";

    double step_;
    void lineCallback(const std_msgs::String::ConstPtr &msg);
    void magnetCallback(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void executeGridSequence();
    bool isInZone(double x, double y);
    void executeLineSequence();
    void sendRelativeGoal(double dx, double dy, double dyaw);
    bool waitForResult() const;
    void publishAbort() const;
    void publishState(const std::string &msg) const;

};


#endif //SEARCHMETA_H
