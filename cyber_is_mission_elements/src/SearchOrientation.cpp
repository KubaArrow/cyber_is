//
// Created by victoria on 15.05.25.
//

#include "cyber_is_mission_elements/SearchOrientation.h"
#include <virtual_costmap_layer/Obstacles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <cmath>


SearchOrientation::SearchOrientation(
    ros::NodeHandle &nh,
    const std::string &state_topic,
    const std::string &odom_topic,
    const std::string &line_detector_topic,
    const std::string &wall_topic)
    : nh_(nh),
      ac_("move_base", true),
      state_topic_(state_topic),
      line_detector_topic_(line_detector_topic),
      wall_topic_(wall_topic),
      odom_topic_(odom_topic) {

    // ───────── parameters ─────────
    nh_.param<std::string>("/mission/start_position", wall_start_, "bottom");
    nh_.param("/mission/move_front", move_front_, 1.5);
    nh_.param("/mission/move_side", move_side_, 1.0);
    nh_.param("/mission/cage_height", cage_height_, 2.50);
    nh_.param("/mission/cage_width", cage_width_, 2.50);
    nh_.param("/mission/cage_size", cage_size_, 0.10);
    nh_.param("/mission/cage_move", cage_move_, 0.10);

    nh_.param("/mission/turn_left", turn_left_, false);


    // ───────── topics ─────────

    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    wall_pub_ = nh_.advertise<virtual_costmap_layer::Obstacles>(wall_topic_, 1);

    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &SearchOrientation::lineCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &SearchOrientation::odomCallback, this);


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

void SearchOrientation::executeSequence() {
    const double yaw = (turn_left_ ? M_PI / 2.0 : -M_PI / 2.0);
    sendRelativeGoal(move_front_, 0.0, yaw);
    if (!waitForResult()) return;

    search_active_ = true;
    sendRelativeGoal(move_side_, 0.0, 0.0);
    if (!waitForResultWithAbordOnDone()) return;
    search_active_ = false;
}

// ————————————————————————————————————————— helpers ——————————————————————————————————————————————
void SearchOrientation::sendRelativeGoal(const double dx, const double dy, const double dyaw) {
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

bool SearchOrientation::waitForResult() const {

    ros::Rate r(20);
    while (ros::ok()) {
        if (full_line_detected_) {
            ROS_WARN("[SearchOrientation] FULL_LINE detected → cancel current goal");

            return false;
        }
        if (ac_.getState().isDone()) {
            return !ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
        }


        ros::spinOnce();
        r.sleep();
    }
    return false;
}

bool SearchOrientation::waitForResultWithAbordOnDone() const {

    ros::Rate r(20);
    while (ros::ok()) {
        if (full_line_detected_) {
            ROS_WARN("[SearchOrientation] FULL_LINE detected → cancel current goal");

            return false;
        }

        if (ac_.getState().isDone()) {
            ROS_WARN("[SearchOrientation] Move vase done -> cancel mission");
            publishAbort();
            return ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
        }


        ros::spinOnce();
        r.sleep();
    }
    return false;
}


// ————————————————————————————————————————— callbacks ————————————————————————————————————————————
void SearchOrientation::lineCallback(const std_msgs::String::ConstPtr &msg) {
    if (!search_active_) return; // ignore until Y‑segment is active

    if (msg->data != "NO_LINE") {
        ac_.cancelAllGoals();

        ROS_INFO("[SearchOrientation] FULL_LINE detected → stop & build virtual wall");
        odom_sub_.shutdown();
        sendRelativeGoal(-move_side_ / 2, 0.0, 0.0);
        if (!waitForResult()) return;
        make_walls();

        publishState( "FOUNDED_ORIENTATION");
    }
}

static geometry_msgs::Point makePoint(double x, double y, double z = 0.0) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

void SearchOrientation::make_walls() {

    geometry_msgs::Point start, orient;
    nh_.getParam("/start_pose/x", start.x);
    nh_.getParam("/start_pose/y", start.y);
    nh_.getParam("/orientation_pose/x", orient.x);
    nh_.getParam("/orientation_pose/y", orient.y);

    if (wall_start_ == "bottom" || wall_start_ == "top") {
        cornerPoint.x = start.x;
        cornerPoint.y = orient.y;
    } else {
        cornerPoint.x = orient.x;
        cornerPoint.y = start.y;
    }

    geometry_msgs::Point vecRight, vecUp; // jednostkowe wektory osi W i H

    if (wall_start_ == "bottom") {
        vecRight = makePoint(0, (turn_left_ ? -1 : 1)); // oś W na Y
        vecUp = makePoint(1, 0); // oś H na X (DO GÓRY!)
    } else if (wall_start_ == "top") {
        vecRight = makePoint(0, (turn_left_ ? 1 : -1));
        vecUp = makePoint(-1, 0);
    } else if (wall_start_ == "left") {
        vecRight = makePoint(1, 0); // W na X
        vecUp = makePoint(0, (turn_left_ ? -1 : 1)); // H na Y
    } else /* "right" */
    {
        vecRight = makePoint(-1, 0);
        vecUp = makePoint(0, (turn_left_ ? 1 : -1));
    }

    const double W = cage_width_;
    const double H = cage_height_;

    auto add = [](const geometry_msgs::Point &a,
                  const geometry_msgs::Point &v,
                  const double s) -> geometry_msgs::Point {
        return makePoint(a.x + v.x * s,
                         a.y + v.y * s);
    };


    const geometry_msgs::Point bl = cornerPoint;
    const geometry_msgs::Point br = add(bl, vecRight, W);
    const geometry_msgs::Point tl = add(bl, vecUp, H);
    const geometry_msgs::Point tr = add(tl, vecRight, W);

    virtual_costmap_layer::Obstacles msg;
    auto addWall = [&](const geometry_msgs::Point &a,
                       const geometry_msgs::Point &b) {
        virtual_costmap_layer::Form wall;
        wall.form = {a, b};
        msg.list.push_back(wall);
    };
    addWall(bl, br);
    addWall(br, tr);
    addWall(tr, tl);
    addWall(tl, bl);

    wall_pub_.publish(msg);

    ROS_INFO_STREAM("[SearchOrientation] Cage made ‑ corner=("
        << cornerPoint.x << ", " << cornerPoint.y << ")");
}



void SearchOrientation::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose_ = msg->pose.pose;
    nh_.setParam("/orientation_pose/x", current_pose_.position.x);
    nh_.setParam("/orientation_pose/y", current_pose_.position.y);
}

void SearchOrientation::publishAbort() const {
    publishState("ABORT_MISSION");
}

void SearchOrientation::publishState(const std::string &msg) const {
    std_msgs::String s;
    s.data = msg;
    state_pub_.publish(s);
}
