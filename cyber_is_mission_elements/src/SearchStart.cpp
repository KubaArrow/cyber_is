//
// Created by victoria on 15.05.25.
//

#include "cyber_is_mission_elements/SearchStart.h"
#include <virtual_costmap_layer/Obstacles.h>
#include <virtual_costmap_layer/Form.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SearchStart::SearchStart(
    ros::NodeHandle &nh,
    const std::string &state_topic,
    const std::string &odom_topic,
    const std::string &line_detector_topic,
    const std::string &wall_topic)
    : nh_(nh),
      ac_("move_base", true),
      state_topic_(state_topic),
      odom_topic_(odom_topic),
      line_detector_topic_(line_detector_topic),
      wall_topic_(wall_topic) {
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    wall_pub_ = nh_.advertise<virtual_costmap_layer::Obstacles>(wall_topic_, 1, true);
    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &SearchStart::lineCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &SearchStart::odomCallback, this);

    search_active_ = true;
    full_line_detected_ = false;

    ROS_INFO("[SearchStart] Waiting for move_base action server...");
    if (!ac_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("[SearchStart] Move_base server not available – aborting mission");
        publishAbort();
        ros::shutdown();
        return;
    }
    ROS_INFO("[SearchStart] Move_base connected");

    executeSequence();
}

void SearchStart::executeSequence() {
    sendRelativeGoal(1.0, 0.0, 0.0);
    if (!waitForResult()) return;
}

void SearchStart::lineCallback(const std_msgs::String::ConstPtr &msg) {
    if (!search_active_) return;

    if (msg->data != "NO_LINE" && !full_line_detected_) {
        full_line_detected_ = true;
        ROS_INFO("[SearchStart] Line detected.");
        odom_sub_.shutdown();
        make_wall();
        publishState("FOUNDED_START_POSE");
    }
}

void SearchStart::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose_ = msg->pose.pose;
    nh_.setParam("/start_pose/x", current_pose_.position.x);
    nh_.setParam("/start_pose/y", current_pose_.position.y);
}


void SearchStart::sendRelativeGoal(const double dx, const double dy, const double dyaw) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_footprint"; // relative frame
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dyaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);

    ac_.sendGoal(goal);
}

bool SearchStart::waitForResult() const {
    ros::Rate r(20);
    while (ros::ok()) {
        if (full_line_detected_) {
            ROS_WARN("[SearchStart] Line detected");
            return false;
        }

        if (ac_.getState().isDone()) {
            publishAbort();
            return ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
        }

        ros::spinOnce();
        r.sleep();
    }
    return false;
}

void SearchStart::make_wall() const {
    geometry_msgs::Point p0, p1, p2, p3;
    p0.x = current_pose_.position.x - 0.30;
    p0.y = -10.0;
    p1.x = current_pose_.position.x - 0.30;
    p1.y = 10.0;
    p2.x = current_pose_.position.x - 0.50;
    p2.y = 10.0;
    p3.x = current_pose_.position.x - 0.50;
    p3.y = -10.0;

    virtual_costmap_layer::Form polygon;
    polygon.form = {p0, p1, p2, p3};

    virtual_costmap_layer::Obstacles msg;
    msg.list.push_back(polygon);

    wall_pub_.publish(msg);
}

void SearchStart::publishAbort() const {
    publishState("ABORT_MISSION");
}

void SearchStart::publishState(const std::string &msg) const {
    std_msgs::String s;
    s.data = msg;
    state_pub_.publish(s);
}
