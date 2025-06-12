#include "cyber_is_mission_elements/SearchMeta.h"

#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <set>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <virtual_costmap_layer/Obstacles.h>

SearchMeta::SearchMeta(
    ros::NodeHandle &nh,
    const std::string &state_topic,
    const std::string &line_detector_topic,
    const std::string &odom_topic,
    const std::string &wall_topic)
    : nh_(nh),
      ac_("move_base", true),
      state_topic_(state_topic),
      odom_topic_(odom_topic),
	  wall_topic_(wall_topic),
      line_detector_topic_(line_detector_topic) {
    XmlRpc::XmlRpcValue pts_param;
    if (!nh.getParam("/mission/zone_points", pts_param) || pts_param.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        pts_param.
        size() != 4) {
        ROS_ERROR("[SearchMeta] Zone_points must be an array of four [x,y] lists");
        ros::shutdown();
        return;
    }
    for (int i = 0; i < pts_param.size(); ++i) {
        XmlRpc::XmlRpcValue pt = pts_param[i];
        if (pt.getType() != XmlRpc::XmlRpcValue::TypeArray || pt.size() != 2) {
            ROS_ERROR("[SearchMeta] Each point must be [x, y]");
            ros::shutdown();
            return;
        }
        double x = pt[0];
        double y = pt[1];
        points_.push_back({x, y});
    }

    magnet_topic_ = nh_.param<std::string>("/mission/magnet_topic", "/magnet_filtered"); // new param
    mode_ = nh_.param<std::string>("/mission/mode", "line"); // new param
    step_ = nh_.param<double>("/mission/magnet_topic", 0.2); // new param

    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &SearchMeta::odomCallback, this);
    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &SearchMeta::lineCallback, this);
    magnet_sub_ = nh_.subscribe(magnet_topic_, 1, &SearchMeta::magnetCallback, this);

    ROS_INFO("[SearchMeta] Waiting for move_base action server...");
    if (!ac_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("[SearchMeta] Move_base server not available – aborting mission");
        publishAbort();
        ros::shutdown();
        return;
    }
    ROS_INFO("[SearchMeta] Move_base connected");

    wall_pub_ = nh_.advertise<virtual_costmap_layer::Obstacles>(wall_topic_, 1, /*latch=*/true);

    virtual_costmap_layer::Obstacles empty_msg;
    wall_pub_.publish(empty_msg);
    ros::spinOnce();

    if (mode_ == "line") {
        executeLineSequence();
    }else {
        executeGridSequence();
    }


}

bool SearchMeta::isInZone(double x, double y) {
    // Prosty algorytm crossing number, działa dla dowolnego prostokąta
    int nvert = points_.size();
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((points_[i].second > y) != (points_[j].second > y)) &&
            (x < (points_[j].first - points_[i].first) * (y - points_[i].second) /
             (points_[j].second - points_[i].second) + points_[i].first))
            c = !c;
    }
    return c;
}


void SearchMeta::executeLineSequence() {
    ROS_INFO("[SearchMeta] Starting simple forward drive until line...");
    search_active_ = true;
    full_line_detected_ = false;

    // przygotuj publisher cmd_vel
    ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // ustaw stałą prędkość
    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;  // np. 0.2 m/s
    twist.angular.z = 0.0;

    ros::Rate rate(20.0);   // 20 Hz

    // jeździmy, aż wykryjemy linię lub rozmijemy się z ros::ok()
    while (ros::ok() && !full_line_detected_) {
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    // zatrzymanie robota
    twist.linear.x = 0.0;
    vel_pub.publish(twist);
    ros::spinOnce();
    ROS_INFO("[SearchMeta] Line detected, forward drive stopped.");

    // ewentualnie można tu dodać kolejne etapy (obroty, inne manewry)
    publishState("ZONE_SEARCHED_LINE");
}




void SearchMeta::lineCallback(const std_msgs::String::ConstPtr &msg) {
     if (!search_active_) return;
    last_line_status_ = msg->data;

    if (msg->data != "NO_LINE") {
        ac_.cancelAllGoals();
        full_line_detected_=true;
        ROS_INFO("[SearchMeta] FULL_LINE detected");
    }
}


void SearchMeta::executeGridSequence() {
    // 1. Pobierz min/max x, y — zakładamy prostokąt
    double min_x = points_[0].first, max_x = points_[0].first;
    double min_y = points_[0].second, max_y = points_[0].second;
    for (const auto &pt: points_) {
        if (pt.first < min_x) min_x = pt.first;
        if (pt.first > max_x) max_x = pt.first;
        if (pt.second < min_y) min_y = pt.second;
        if (pt.second > max_y) max_y = pt.second;
    }

    // 2. Utwórz grid
    std::vector<std::pair<double, double> > grid_points;
    for (double x = min_x + step_; x <= max_x - step_ / 2; x += step_) {
        for (double y = min_y + step_; y <= max_y - step_ / 2; y += step_) {
            grid_points.emplace_back(x, y);
        }
    }
    ROS_INFO("[SearchMeta] Created grid with %lu points (step=%.2f)", grid_points.size(), step_);

    // 3. Zbiór nieodwiedzonych punktów
    std::vector<std::pair<double, double> > unvisited = grid_points;

    search_active_ = true;
    found_ = false;

    // 4. Punkt startowy — aktualna pozycja robota
    double robot_x = current_pose_.position.x;
    double robot_y = current_pose_.position.y;

    while (!unvisited.empty() && search_active_) {
        // a) Najpierw szukaj najbliższego punktu na prawo od robota (większe X)
        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < unvisited.size(); ++i) {
            double dx = unvisited[i].first - robot_x;
            double dy = unvisited[i].second - robot_y;
            if (dx >= 0.0) {
                // preferuj na prawo od robota (x większe)
                double dist = std::hypot(dx, dy);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = i;
                }
            }
        }
        // b) Jeśli nie ma żadnego na prawo, wybierz dowolny najbliższy
        if (best_idx == -1) {
            for (size_t i = 0; i < unvisited.size(); ++i) {
                double dx = unvisited[i].first - robot_x;
                double dy = unvisited[i].second - robot_y;
                double dist = std::hypot(dx, dy);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = i;
                }
            }
        }
        if (best_idx == -1) break; // powinno się nie zdarzyć

        auto pt = unvisited[best_idx];
        unvisited.erase(unvisited.begin() + best_idx);


        // c) Wysyłaj goal do move_base
        double yaw = atan2(pt.second - robot_y, pt.first - robot_x); // Różnica: do punktu z obecnej pozycji

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = pt.first;
        goal.target_pose.pose.position.y = pt.second;

        // Ustaw orientację w kierunku jazdy!
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        ROS_INFO("[SearchMeta] Sending goal: x=%.2f y=%.2f yaw=%.2f deg", pt.first, pt.second, yaw * 180.0 / M_PI);

        ac_.sendGoal(goal);

        // d) Czekaj na wynik lub przerwanie
        bool finished = false;
        while (ros::ok() && !finished) {
            if (!search_active_) {
                ac_.cancelAllGoals();
                ROS_WARN("[SearchMeta] Search aborted by event");
                return;
            }
            auto state = ac_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                finished = true;
                robot_x = pt.first;
                robot_y = pt.second;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED ||
                       state == actionlib::SimpleClientGoalState::REJECTED) {
                ROS_WARN("[SearchMeta] Couldn't reach grid point, skipping...");
                finished = true;
                // zostawiamy robot_x, robot_y bez zmiany
            }
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
    }

    search_active_ = false;
    if (!found_) {
        ROS_INFO("[SearchMeta] Finished searching zone. No meta found.");
        publishState("ZONE_SEARCHED_NO_META");
        publishAbort();
    }
}


void SearchMeta::magnetCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (!search_active_) return; // ignore until Y‑segment is active
    if (msg->data) {

        search_active_ = false;
        ROS_INFO("[SearchMeta] Meta detected");
        ac_.cancelAllGoals();
        publishState("FOUNDED_FINISH");
    }
}

bool SearchMeta::waitForResult() const {

    ros::Rate r(20);
    while (ros::ok()) {
        if (full_line_detected_) {
            ROS_WARN("[SearchOrientation] FULL_LINE detected");

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

void SearchMeta::sendRelativeGoal(const double dx, const double dy, const double dyaw) {
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

void SearchMeta::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose_ = msg->pose.pose;
}

void SearchMeta::publishAbort() const { publishState("ABORT_MISSION"); }

void SearchMeta::publishState(const std::string &msg) const {
    std_msgs::String s;
    s.data = msg;
    state_pub_.publish(s);
}
