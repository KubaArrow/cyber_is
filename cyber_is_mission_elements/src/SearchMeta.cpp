// SearchMeta.cpp
#include "cyber_is_mission_elements/SearchMeta.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <virtual_costmap_layer/Obstacles.h>
#include <cmath>
#include <tf2/utils.h>   // dla tf2::getYaw


SearchMeta::SearchMeta(
    ros::NodeHandle &nh,
    const std::string &state_topic,
    const std::string &line_detector_topic,
    const std::string &odom_topic,
    const std::string &wall_topic)
    : nh_(nh),
      ac_("move_base", true),
      state_topic_(state_topic),
      line_detector_topic_(line_detector_topic),
      odom_topic_(odom_topic),
      wall_topic_(wall_topic) {
    // Load zone corners
    XmlRpc::XmlRpcValue zone_param;
    if (!nh_.getParam("/mission/zone_points", zone_param) ||
        zone_param.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        zone_param.size() != 4) {
        ROS_ERROR("[SearchMeta] zone_points must be 4 [x,y] pairs");
        ros::shutdown();
        return;
    }
    for (int i = 0; i < zone_param.size(); ++i) {
        auto &pt = zone_param[i];
        points_.emplace_back((double)pt[0], (double)pt[1]);
    }

    // Load sample points
    XmlRpc::XmlRpcValue samples_param;
    if (!nh_.getParam("/mission/samples_points", samples_param) ||
        samples_param.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        samples_param.size() == 0) {
        ROS_ERROR("[SearchMeta] samples_points must be a non-empty array");
        ros::shutdown();
        return;
    }
    for (int i = 0; i < samples_param.size(); ++i) {
        auto &pt = samples_param[i];
        samples_points_.emplace_back((double)pt[0], (double)pt[1]);
    }

    // Other params
    magnet_topic_ = nh_.param<std::string>("/mission/magnet_topic", "/magnet_filtered");
    mode_         = nh_.param<std::string>("/mission/search_mode", "sampling");
    move_cage_    = nh_.param<double>("/mission/cage_move_search", 0.0);
    final_orientation_ = nh_.param<std::string>("/mission/final_orientation", "none");

    // Publishers & subscribers
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    wall_pub_  = nh_.advertise<virtual_costmap_layer::Obstacles>(wall_topic_, 1, true);
    vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    odom_sub_  = nh_.subscribe(odom_topic_, 1, &SearchMeta::odomCallback, this);
    line_sub_  = nh_.subscribe(line_detector_topic_, 1, &SearchMeta::lineCallback, this);
    magnet_sub_= nh_.subscribe(magnet_topic_, 1, &SearchMeta::magnetCallback, this);

    // Clear walls
    virtual_costmap_layer::Obstacles empty;
    wall_pub_.publish(empty);
    ros::spinOnce();

    ROS_INFO("[SearchMeta] Waiting for move_base server...");
    if (!ac_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("[SearchMeta] move_base not available");
        publishAbort(); ros::shutdown(); return;
    }
    ROS_INFO("[SearchMeta] Connected to move_base");

    // Start sequence
    if (mode_ == "line") executeLineSequence();
    else                 executeSamplingSequence();
}

void SearchMeta::executeSamplingSequence()
{
    ROS_INFO("[SearchMeta] Starting sampling sequence");
    sampling_active_ = true;
    ros::Rate rate(20.0);

    std::vector<bool> visited(samples_points_.size(), false);
    bool any_reached = false;

    /* -----------------------------------------------------------
     *  Główna pętla – dopóki aktywne i są nieodwiedzone próbki
     * ----------------------------------------------------------*/
    while (sampling_active_)
    {
        /*–– 1. Wybór najbliższej nieodwiedzonej próbki ––*/
        size_t idx = findNearestSample(current_pose_, visited);
        if (visited[idx]) break;                  // wszystko zaliczone
        visited[idx] = true;

        const auto& sp = samples_points_[idx];
        double sx  = sp.first;
        double sy  = sp.second;
        double yaw = yawFromFinalOrientation();

        /*–– 2. Dojazd do punktu próbki ––*/
        ROS_INFO_STREAM("[SearchMeta] Navigating to sample (" << sx << ", " << sy << ")");
        sendGoal(sx, sy, yaw);

        if (!waitForResult(180.0)) {              // 3 minuty na dojazd
            ROS_WARN_STREAM("[SearchMeta] Navigation to sample " << idx << " failed, skipping");
            continue;
        }
        any_reached = true;

        /*–– 3. Lekki podjazd, aż zobaczymy linię ––*/
        geometry_msgs::Pose start_pose = current_pose_;   // zapamiętujemy
        full_line_detected_ = false;

        while (ros::ok() && !full_line_detected_ && sampling_active_) {
            geometry_msgs::Twist t;  t.linear.x = 0.1;
            vel_pub_.publish(t);
            ros::spinOnce();  rate.sleep();
        }
        ros::Duration(1.0).sleep();
        stopRobot();

        /*–– 4. Powrót do start_pose przez move_base ––*/
        if (full_line_detected_) {
            ROS_INFO("[SearchMeta] Line detected, returning to start pose via move_base");

            double start_yaw = tf2::getYaw(start_pose.orientation);
            sendGoal(start_pose.position.x,
                     start_pose.position.y,
                     start_yaw);

            if (!waitForResult(60.0)) {
                ROS_WARN("[SearchMeta] Return to start pose failed – cancelling");
                ac_.cancelGoal();
            }
            stopRobot();
        } else {
            ROS_WARN_STREAM("[SearchMeta] No line detected at sample " << idx);
        }

        /*–– 5. Czy zostały próbek? ––*/
        if (std::all_of(visited.begin(), visited.end(), [](bool v){ return v; }))
            break;
    }

    sampling_active_ = false;
    if (!any_reached) {
        ROS_ERROR("[SearchMeta] No samples reached, aborting mission");
        publishAbort();
    } else {
        ROS_INFO("[SearchMeta] Sampling sequence complete");
        publishState("SAMPLING_COMPLETE");
    }
}


size_t SearchMeta::findNearestSample(
    const geometry_msgs::Pose &pose,
    const std::vector<bool> &visited) const {
    size_t best = 0;
    double mind = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < samples_points_.size(); ++i) {
        if (visited[i]) continue;
        auto [x,y] = samples_points_[i];
        double d2 = std::hypot(x - pose.position.x, y - pose.position.y);
        if (d2 < mind) { mind = d2; best = i; }
    }
    return best;
}

double SearchMeta::yawFromFinalOrientation() const
{
    if (points_.size() != 4)                     // zabezpieczenie
        return std::numeric_limits<double>::quiet_NaN();

    /* 1. Centroid strefy */
    double cx = 0, cy = 0;
    for (auto &p : points_) { cx += p.first; cy += p.second; }
    cx /= 4.0;  cy /= 4.0;

    /* 2. Środki krawędzi prostokąta */
    struct P { double x, y; };
    std::array<P,4> edge_centers;
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        edge_centers[i] = {(points_[i].first + points_[j].first) * 0.5,
                           (points_[i].second + points_[j].second) * 0.5};
    }

    /* 3. Wybór krawędzi zgodnie z final_orientation_ */
    double target_x = cx, target_y = cy;   // domyślnie centroid
    if (final_orientation_ == "right") {
        size_t idx = 0; double max_y = edge_centers[0].y;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].y > max_y) { max_y = edge_centers[i].y; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "top") {
        size_t idx = 0; double max_x = edge_centers[0].x;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].x > max_x) { max_x = edge_centers[i].x; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "left") {
        size_t idx = 0; double min_y = edge_centers[0].y;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].y < min_y) { min_y = edge_centers[i].y; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else if (final_orientation_ == "bottom") {
        size_t idx = 0; double min_x = edge_centers[0].x;
        for (size_t i = 1; i < edge_centers.size(); ++i)
            if (edge_centers[i].x < min_x) { min_x = edge_centers[i].x; idx = i; }
        target_x = edge_centers[idx].x;
        target_y = edge_centers[idx].y;

    } else {    // "none" lub literówka
        return std::numeric_limits<double>::quiet_NaN();
    }

    /* 4. Yaw – wektor od punktu docelowego do centroidu */
    return std::atan2(cy - target_y, cx - target_x);
}


void SearchMeta::stopRobot() {
    geometry_msgs::Twist t; vel_pub_.publish(t);
}

void SearchMeta::reverseToStart(const geometry_msgs::Pose &start, double speed) {
    ros::Rate rate(20.0);
    while (ros::ok()) {
        double d = std::hypot(current_pose_.position.x - start.position.x,
                              current_pose_.position.y - start.position.y);
        if (d < 0.05) break;
        geometry_msgs::Twist t; t.linear.x = -speed;
        vel_pub_.publish(t);
        ros::spinOnce(); rate.sleep();
    }
}


bool SearchMeta::waitForResult(double timeout_sec /* = 60.0 */)
{
    bool finished = ac_.waitForResult(ros::Duration(timeout_sec));      // <-- czekamy na odpowiedź
    if (!finished) {                                                    // timeout
        ROS_WARN("[SearchMeta] Goal timeout – cancelling");
        ac_.cancelGoal();
        return false;
    }

    auto st = ac_.getState();
    if (st == actionlib::SimpleClientGoalState::SUCCEEDED)  return true;

    ROS_WARN_STREAM("[SearchMeta] Goal aborted: " << st.toString());
    return false;
}

geometry_msgs::Point SearchMeta::makePoint(double x, double y) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    return p;
}


void SearchMeta::sendGoal(double x, double y, double yaw) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
    ac_.sendGoal(goal);
}

void SearchMeta::sendRelativeGoal(double dx, double dy, double dyaw) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;
    tf2::Quaternion q; q.setRPY(0, 0, dyaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
    ac_.sendGoal(goal);
}

void SearchMeta::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose_ = msg->pose.pose;
}

void SearchMeta::lineCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data != "NO_LINE") {
        full_line_detected_ = true;
        ac_.cancelAllGoals();
        ROS_INFO("[SearchMeta] FULL_LINE detected");
    }
}

void SearchMeta::magnetCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        sampling_active_ = false;
        ROS_INFO("[SearchMeta] Meta detected");
        publishState("FOUNDED_FINISH");
        ros::spinOnce();
        ac_.cancelAllGoals();
        stopRobot();
    }
}

void SearchMeta::publishAbort() const {
    publishState("ABORT_MISSION");
}

void SearchMeta::publishState(const std::string &msg) const {
    std_msgs::String s; s.data = msg;
    state_pub_.publish(s);
}

void SearchMeta::executeLineSequence() {
    ROS_INFO("[SearchMeta] Starting line sequence");
    search_active_ = true;
    full_line_detected_ = false;

    // 1. Drive forward until line detected
    ros::Rate rate(20);
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.1;
    while (ros::ok() && !full_line_detected_ && search_active_) {
        vel_pub_.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
    ros::Duration(1.0).sleep();
    stopRobot();
    if (!full_line_detected_) {
        ROS_WARN("[SearchMeta] Line not detected");
        publishAbort();
        return;
    }

    // 2. Rotate 90 degrees
    sendRelativeGoal(0.0, 0.0, M_PI_2);
    waitForResult(5.0);

    // 3. Build cage around orientation point
    double cage_h = nh_.param<double>("/mission/cage_height", 2.5);
    double cage_w = nh_.param<double>("/mission/cage_width", 2.5);
    double cage_size = nh_.param<double>("/mission/cage_size", 0.1);
    geometry_msgs::Point start, orient;
    nh_.getParam("/start_pose/x", start.x);
    nh_.getParam("/start_pose/y", start.y);
    nh_.getParam("/orientation_pose/x", orient.x);
    nh_.getParam("/orientation_pose/y", orient.y);
    geometry_msgs::Point corner;
    if (final_orientation_ == "bottom" || final_orientation_ == "top") {
        corner.x = start.x;
        corner.y = orient.y;
    } else {
        corner.x = orient.x;
        corner.y = start.y;
    }
    nh_.setParam("/corner_pose/x", corner.x);
    nh_.setParam("/corner_pose/y", corner.y);

    bool turn_left = true;
    geometry_msgs::Point vecRight, vecUp;
    if (final_orientation_ == "bottom") {
        vecRight = makePoint(0, turn_left ? -1 : 1);
        vecUp = makePoint(1, 0);
    } else if (final_orientation_ == "top") {
        vecRight = makePoint(0, turn_left ? 1 : -1);
        vecUp = makePoint(-1, 0);
    } else if (final_orientation_ == "left") {
        vecRight = makePoint(1, 0);
        vecUp = makePoint(0, turn_left ? -1 : 1);
    } else {
        vecRight = makePoint(-1, 0);
        vecUp = makePoint(0, turn_left ? 1 : -1);
    }

    double W = cage_w + 2 * move_cage_;
    double H = cage_h + 2 * move_cage_;
    auto add = [&](const geometry_msgs::Point &a, const geometry_msgs::Point &v, double s) {
        return makePoint(a.x + v.x * s, a.y + v.y * s);
    };
    auto bl = add(add(corner, vecRight, -move_cage_), vecUp, -move_cage_);
    auto br = add(bl, vecRight, W);
    auto tl = add(bl, vecUp, H);
    auto tr = add(tl, vecRight, W);

    virtual_costmap_layer::Obstacles obs;
    obs.list.clear();
    auto addWall = [&](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
        virtual_costmap_layer::Form f;
        f.form = {a, b};
        obs.list.push_back(f);
    };
    addWall(bl, br); addWall(br, tr); addWall(tr, tl); addWall(tl, bl);
    wall_pub_.publish(obs);

    // 4. Traverse within shifted zone
    geometry_msgs::Point zmin, zmax;
    zmin.x = zmin.y = std::numeric_limits<double>::infinity();
    zmax.x = zmax.y = -std::numeric_limits<double>::infinity();
    for (auto &p : points_) {
        double x = p.first + corner.x;
        double y = p.second + corner.y;
        zmin.x = std::min(zmin.x, x);
        zmin.y = std::min(zmin.y, y);
        zmax.x = std::max(zmax.x, x);
        zmax.y = std::max(zmax.y, y);
    }
    ROS_INFO_STREAM("Zone X:[" << zmin.x << "," << zmax.x << "] Y:[" << zmin.y << "," << zmax.y << "]");

    bool forward = true;
    std::vector<double> deltas{0.5, 0.75, 1.0, 1.25, 1.5};
    while (ros::ok() && search_active_) {
        for (double d : deltas) {
            if (!search_active_) break;
            double dir = forward ? d : -d;
            double tx = current_pose_.position.x + vecRight.x * dir;
            double ty = current_pose_.position.y + vecRight.y * dir;
            if (tx < zmin.x || tx > zmax.x || ty < zmin.y || ty > zmax.y) continue;
            sendGoal(tx, ty, std::numeric_limits<double>::quiet_NaN());
            if (waitForResult(10.0)) break;
            ROS_WARN_STREAM("Step " << d << " failed, skipping");
        }
        double pos = vecRight.x ? current_pose_.position.x : current_pose_.position.y;
        double bound = forward ? (vecRight.x ? zmax.x : zmax.y) : (vecRight.x ? zmin.x : zmin.y);
        if (std::fabs(pos - bound) < 0.1) forward = !forward;
        ros::spinOnce();
    }

    stopRobot();
    ROS_INFO("[SearchMeta] Line sequence complete");
}
