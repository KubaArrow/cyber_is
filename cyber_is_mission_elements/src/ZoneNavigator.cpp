//
// Created by victoria on 15.05.25.
//

#include "cyber_is_mission_elements/ZoneNavigator.h"
#include <geometry_msgs/PolygonStamped.h>
#include <actionlib/client/terminal_state.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <limits>



ZoneNavigator::ZoneNavigator(ros::NodeHandle &nh, const std::string &state_topic)
    : nh_(nh), ac_("move_base", true), state_topic_(state_topic) {
    // ------------------- parameters -------------------
    XmlRpc::XmlRpcValue pts_param;
    if (!nh.getParam("/mission/zone_points", pts_param) || pts_param.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        pts_param.
        size() != 4) {
        ROS_ERROR("[ZoneNavigator] Zone_points must be an array of four [x,y] lists");
        ros::shutdown();
        return;
    }

    zone_topic_ = nh_.param<std::string>("/mission/zone_topic", "/zone_polygon"); // new param
    final_orientation_ = nh_.param<std::string>("/mission/final_orientation", "none"); // new param
    ROS_INFO_STREAM("[ZoneNavigator] Final_orientation : " << final_orientation_);
    // static const std::unordered_map<std::string, double> orient_map = {
    //     {"none", std::numeric_limits<double>::quiet_NaN()},
    //     {"top", 0.0},
    //     {"right", -M_PI_2},
    //     {"bottom", M_PI},
    //     {"left", M_PI_2}
    // };

    // auto it = orient_map.find(final_orientation_);
    // if (it == orient_map.end()) {
    //     ROS_WARN_STREAM("[ZoneNavigator] Unknown final_orientation '" << final_orientation_ << "' – falling back to 'none'");
    //     final_orientation_ = "none";
    // }

    // convert points
    for (int i = 0; i < pts_param.size(); ++i) {
        XmlRpc::XmlRpcValue pt = pts_param[i];
        if (pt.getType() != XmlRpc::XmlRpcValue::TypeArray || pt.size() != 2) {
            ROS_ERROR("[ZoneNavigator] Each point must be [x, y]");
            ros::shutdown();
            return;
        }
        double x = pt[0];
        double y = pt[1];
        points_.push_back({x, y});
    }

    // ------------------- publishers / action client -------------------
    poly_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(zone_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>("/robot_state", 1, true);

    ROS_INFO("[ZoneNavigator] Waiting for move_base action server...");
    if (!ac_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("[ZoneNavigator] Move_base server not available – aborting mission");
        publishAbort();
        ros::shutdown();
        return;
    }
    ROS_INFO("[ZoneNavigator] Move_base connected");

    publishPolygon();
    sendGoalAndWait();
}


void ZoneNavigator::publishPolygon() const {
    geometry_msgs::PolygonStamped poly;
    poly.header.stamp = ros::Time::now();
    poly.header.frame_id = "map";
    for (const auto &p: points_) {
        geometry_msgs::Point32 pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0;
        poly.polygon.points.push_back(pt);
    }
    poly_pub_.publish(poly);
    ROS_INFO_STREAM("[ZoneNavigator] Zone polygon published on "<<zone_topic_);
}

std::pair<double, double> ZoneNavigator::centroid() const {
    double sx = 0, sy = 0;
    for (const auto &p: points_) {
        sx += p.first;
        sy += p.second;
    }
    return {sx / points_.size(), sy / points_.size()};
}

// yaw in radians -> quaternion
geometry_msgs::Quaternion ZoneNavigator::yawToQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::Quaternion q_msg;
    tf2::convert(q, q_msg);
    return q_msg;
}

// void ZoneNavigator::sendGoalAndWait() {
//     auto c = centroid();
//     ROS_INFO_STREAM(
//         "[ZoneNavigator] Sending robot to zone center (" << c.first << ", " << c.second << ") in map with orientation '"
//         << final_orientation_ << "'");
//
//     move_base_msgs::MoveBaseGoal goal;
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();
//     goal.target_pose.pose.position.x = c.first;
//     goal.target_pose.pose.position.y = c.second;
//     goal.target_pose.pose.position.z = 0.0;
//
//     // orientation
//     double yaw = std::numeric_limits<double>::quiet_NaN();
//     if (final_orientation_ == "top") yaw = 0.0;
//     else if (final_orientation_ == "right") yaw = -M_PI_2;
//     else if (final_orientation_ == "left") yaw = M_PI_2;
//     else if (final_orientation_ == "bottom") yaw = M_PI;
//
//     if (std::isnan(yaw)) {
//         goal.target_pose.pose.orientation.w = 1.0; // keep current orientation (identity)
//     } else {
//         goal.target_pose.pose.orientation = yawToQuaternion(yaw);
//     }
//     ROS_INFO_STREAM("[ZoneNavigator] Send goal: " << yaw);
//     ac_.sendGoal(goal);
//
//     bool finished = ac_.waitForResult();
//     if (!finished) {
//         ROS_WARN("[ZoneNavigator] Navigation timed out – aborting mission");
//         ac_.cancelGoal();
//         publishAbort();
//         return;
//     }
//
//     auto state = ac_.getState();
//     if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
//         ROS_INFO("[ZoneNavigator] Robot reached zone – success");
//         publishState("FOUNDED_ZONE");
//     } else {
//         ROS_ERROR_STREAM("[ZoneNavigator] Navigation failed: " << state.toString());
//         publishAbort();
//     }
// }

void ZoneNavigator::sendGoalAndWait()
{
    if (points_.size() != 4) {
        ROS_ERROR("[ZoneNavigator] Zone must have exactly 4 points!");
        publishAbort();
        return;
    }

    //--------------------------------------------
    // 1. Pobranie narożnika z parametrów
    //--------------------------------------------
    double corner_x = 0.0, corner_y = 0.0;
    bool have_corner =
        nh_.getParam("/corner_pose/x", corner_x) &&
        nh_.getParam("/corner_pose/y", corner_y);

    if (have_corner) {
        ROS_INFO_STREAM("[ZoneNavigator] corner_pose = (" << corner_x
                        << ", " << corner_y << ")");
    } else {
        ROS_WARN("[ZoneNavigator] corner_pose param not found – "
                 "using zone points as-is");
    }

    //--------------------------------------------
    // 2. Geometria strefy
    //--------------------------------------------
    std::vector<EdgeCenter> edge_centers;
    std::pair<double, double> centroid;
    computeCentersAndCentroid(edge_centers, centroid);

    // jeżeli mamy narożnik – przesuwamy całą geometrię o (corner_x , corner_y)
    if (have_corner) {
        for (auto &ec : edge_centers) {
            ec.x += corner_x;
            ec.y += corner_y;
        }
        centroid.first  += corner_x;
        centroid.second += corner_y;
    }

    //--------------------------------------------
    // 3. Wybór punktu docelowego
    //--------------------------------------------
    double target_x = centroid.first;
    double target_y = centroid.second;

    if (final_orientation_ == "right") {
        size_t idx = 0;
        double max_y = edge_centers[0].y;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].y > max_y) { max_y = edge_centers[i].y; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "top") {
        size_t idx = 0;
        double max_x = edge_centers[0].x;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].x > max_x) { max_x = edge_centers[i].x; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "left") {
        size_t idx = 0;
        double min_y = edge_centers[0].y;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].y < min_y) { min_y = edge_centers[i].y; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "bottom") {
        size_t idx = 0;
        double min_x = edge_centers[0].x;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].x < min_x) { min_x = edge_centers[i].x; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;
    }

    //--------------------------------------------
    // 4. Kierunek – od punktu docelowego do CENTROIDU
    //    (lub do innego punktu odniesienia, jeżeli tak wolisz)
    //--------------------------------------------
    double yaw = std::atan2(centroid.second - target_y,
                            centroid.first  - target_x);

    ROS_INFO_STREAM("[ZoneNavigator] target=(" << target_x << ", "
                    << target_y << ")  yaw=" << yaw);

    //--------------------------------------------
    // 5. Wysyłanie celu do move_base
    //--------------------------------------------
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();
    goal.target_pose.pose.position.x = target_x;
    goal.target_pose.pose.position.y = target_y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = yawToQuaternion(yaw);

    ac_.sendGoal(goal);

    bool finished = ac_.waitForResult();
    if (!finished) {
        ROS_WARN("[ZoneNavigator] Navigation timed out – aborting mission");
        ac_.cancelGoal();
        publishAbort();
        return;
    }

    auto state = ac_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[ZoneNavigator] Robot reached selected point – success");
        publishState("FOUNDED_ZONE");
    } else {
        ROS_ERROR_STREAM("[ZoneNavigator] Navigation failed: " << state.toString());
        publishAbort();
    }
}



void ZoneNavigator::publishAbort() const { publishState("ABORT_MISSION"); }

void ZoneNavigator::publishState(const std::string &msg) const {
    std_msgs::String s;
    s.data = msg;
    state_pub_.publish(s);
    ros::spinOnce();
}

void ZoneNavigator::computeCentersAndCentroid(
    std::vector<EdgeCenter>& edge_centers, std::pair<double, double>& centroid)
{
    edge_centers.clear();
    double sx = 0, sy = 0;
    for (size_t i = 0; i < points_.size(); ++i) {
        size_t j = (i + 1) % points_.size();
        // Środek krawędzi
        double cx = 0.5 * (points_[i].first + points_[j].first);
        double cy = 0.5 * (points_[i].second + points_[j].second);
        double dx = points_[j].first - points_[i].first;
        double dy = points_[j].second - points_[i].second;
        edge_centers.push_back({cx, cy, dx, dy});
        sx += points_[i].first;
        sy += points_[i].second;
    }
    centroid = {sx / points_.size(), sy / points_.size()};
}

