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
    have_corner = nh_.getParam("/corner_pose/x", cornerPoint.x) && nh_.getParam("/corner_pose/y", cornerPoint.y);
        make_walls();
    // Start sequence
    if (mode_ == "line") executeLineSequence();
    else                 executeSamplingSequence();
}

void SearchMeta::executeSamplingSequence()
{
    // 1. Wczytaj narożnik z parametrów i oblicz przekształcone próbki
    std::vector<std::pair<double,double>> local_samples;
    local_samples.reserve(samples_points_.size());
    for (const auto& op : samples_points_) {
        double sx = op.first + (have_corner ? cornerPoint.x : 0.0);
        double sy = op.second + (have_corner ? cornerPoint.y : 0.0);
        local_samples.emplace_back(sx, sy);
    }
    if (have_corner) {
        ROS_INFO_STREAM("[SearchMeta] Local samples offset by corner: x+" << cornerPoint.x << ", y+" << cornerPoint.y);
    } else {
        ROS_WARN("[SearchMeta] corner_pose not set – using raw sample coordinates");
    }

    // 2. Poczekaj na inicjalizację odometrii
    ROS_INFO("[SearchMeta] Initializing odometry...");
    ros::Rate init_rate(10);
    for (int i = 0; i < 20 && ros::ok(); ++i) {
        ros::spinOnce();
        init_rate.sleep();
    }

    // 3. Iteracja po wszystkich punktach po kolei
    ros::Rate rate(20);
    bool any_reached = false;

    for (size_t idx = 0; idx < local_samples.size(); ++idx) {
        double sx = local_samples[idx].first;
        double sy = local_samples[idx].second;
        ROS_INFO_STREAM("[SearchMeta] Sample idx=" << idx << " at (" << sx << ", " << sy << ")");

        // 3.1 Nawigacja do próbki
        double yaw = yawFromFinalOrientation();
        if (!std::isnan(yaw))
            ROS_INFO_STREAM("[SearchMeta] Navigating with yaw=" << yaw);
        sendGoal(sx, sy, yaw);
        if (!waitForResult(30.0)) {
            ROS_WARN_STREAM("[SearchMeta] Navigation to sample " << idx << " failed");
            continue;
        }
        any_reached = true;

        // 3.2 Jazda powolna do przodu aż wykryje linię
        full_line_detected_ = false;
        ROS_INFO("[SearchMeta] Driving slowly until line detection...");
        geometry_msgs::Twist slow;
        slow.linear.x = 0.05;
        while (ros::ok() && !full_line_detected_) {
            vel_pub_.publish(slow);
            ros::spinOnce();
            rate.sleep();
        }
        ros::Duration(0.5).sleep();
        stopRobot();

        // 3.3 Powrót do punktu, na którym dotarliśmy
        if (full_line_detected_) {
            ROS_INFO_STREAM("[SearchMeta] Line detected – returning to sample (" << sx << ", " << sy << ")");
            sendGoal(sx, sy, yaw);
            if (!waitForResult(60.0))
                ROS_WARN("[SearchMeta] Return to sample failed");
            stopRobot();
        } else {
            ROS_WARN_STREAM("[SearchMeta] No line detected at sample " << idx);
        }
    }

    // 4. Zakończenie sekwencji
    if (!any_reached) {
        ROS_ERROR("[SearchMeta] No samples reached – abort mission");
        publishAbort();
    } else {
        ROS_INFO("[SearchMeta] Sampling complete");
        publishState("ABORT_MISSION");
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
    geometry_msgs::Twist t; vel_pub_.publish(t); ros::spinOnce();
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
    ROS_INFO("[SearchMeta] Starting line sequence (move_base sampling)");
    search_active_ = true;

    // 1. Oblicz corner offset i centroid poligonu
    double corner_x = 0.0, corner_y = 0.0;
    nh_.param("/corner_pose/x", corner_x, 0.0);
    nh_.param("/corner_pose/y", corner_y, 0.0);
    double sx = 0.0, sy = 0.0;
    for (const auto &p : points_) {
        sx += p.first + corner_x;
        sy += p.second + corner_y;
    }
    double cx = sx / points_.size();
    double cy = sy / points_.size();
    ROS_INFO_STREAM("Centroid: (" << cx << ", " << cy << ")");

    // 2. Dojazd do centroidu przez move_base
    move_base_msgs::MoveBaseGoal center_goal;
    center_goal.target_pose.header.frame_id = "map";
    center_goal.target_pose.header.stamp = ros::Time::now();
    center_goal.target_pose.pose.position.x = cx;
    center_goal.target_pose.pose.position.y = cy;
    center_goal.target_pose.pose.orientation.w = 1.0;
    ac_.sendGoal(center_goal);
    if (!ac_.waitForResult(ros::Duration(30.0)) ||
        ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("[SearchMeta] Cannot reach centroid");
        publishAbort();
        return;
    }

    // 3. Znajdź dwa najkrótsze brzegi i ich środki
    std::vector<geometry_msgs::Point> verts;
    for (auto &p : points_) {
        geometry_msgs::Point v;
        v.x = p.first + corner_x;
        v.y = p.second + corner_y;
        verts.push_back(v);
    }
    struct Edge { geometry_msgs::Point a,b; double len; };
    std::vector<Edge> edges;
    for (size_t i = 0; i < verts.size(); ++i) {
        auto &a = verts[i];
        auto &b = verts[(i+1)%verts.size()];
        edges.push_back({a,b, std::hypot(b.x - a.x, b.y - a.y)});
    }
    std::sort(edges.begin(), edges.end(), [](auto &e1, auto &e2){ return e1.len < e2.len; });
    geometry_msgs::Point start_pt;
    start_pt.x = (edges[0].a.x + edges[0].b.x) / 2;
    start_pt.y = (edges[0].a.y + edges[0].b.y) / 2;
    start_pt.z = 0.0;
    geometry_msgs::Point end_pt;
    end_pt.x = (edges[1].a.x + edges[1].b.x) / 2;
    end_pt.y = (edges[1].a.y + edges[1].b.y) / 2;
    end_pt.z = 0.0;
    ROS_INFO_STREAM("Start edge center: ("<<start_pt.x<<","<<start_pt.y<<") end: ("<<end_pt.x<<","<<end_pt.y<<")");

    // 4. Sample punkty co 0.2m wzdłuż linii start_pt->end_pt
    double total = std::hypot(end_pt.x - start_pt.x, end_pt.y - start_pt.y);
    size_t N = static_cast<size_t>(std::floor(total/0.2));
    std::vector<geometry_msgs::Point> samples;
    for (size_t i = 0; i <= N; ++i) {
        double t = (double)i / N;
        geometry_msgs::Point sp;
        sp.x = start_pt.x + t*(end_pt.x - start_pt.x);
        sp.y = start_pt.y + t*(end_pt.y - start_pt.y);
        sp.z = 0.0;
        samples.push_back(sp);
    }

    // 5. Nawiguj po kolejnych próbkach: wybieraj najbliższą z unvisited
    std::vector<bool> visited(samples.size(), false);
    size_t visited_count = 0;
    while (ros::ok() && search_active_ && visited_count < samples.size()) {
        // odśwież pozycję
        ros::spinOnce();
        double curx = current_pose_.position.x;
        double cury = current_pose_.position.y;
        // znajdź najbliższą nieodwiedzoną próbkę
        double bestd = std::numeric_limits<double>::infinity();
        size_t besti = 0;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (visited[i]) continue;
            double d = std::hypot(samples[i].x - curx, samples[i].y - cury);
            if (d < bestd) {
                bestd = d;
                besti = i;
            }
        }
        // wyślij cel
        auto &pt = samples[besti];
        move_base_msgs::MoveBaseGoal g;
        g.target_pose.header.frame_id = "map";
        g.target_pose.header.stamp = ros::Time::now();
        g.target_pose.pose.position = pt;
        g.target_pose.pose.orientation.w = 1.0;
        ac_.sendGoal(g);
        bool ok = ac_.waitForResult(ros::Duration(10.0)) &&
                  ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
        visited[besti] = true;
        visited_count++;
        if (ok) ROS_INFO_STREAM("Reached sample "<<besti);
        else ROS_WARN_STREAM("Failed sample "<<besti);
    }

    stopRobot();
    ROS_INFO("[SearchMeta] Line sequence complete");
}







void SearchMeta::make_walls() {
    std::string  wall_start_;
    double cage_height_ = 1.0; // [m]
    double cage_width_ = 1.0; // [m]
    double cage_size_ = 1.0; // [m]
    double cage_move_ = 1.0; // [m]
    bool turn_left_ = true;
    nh_.param<std::string>("/mission/start_position", wall_start_, "bottom");
    nh_.param("/mission/cage_height", cage_height_, 2.50);
    nh_.param("/mission/cage_width", cage_width_, 2.50);
    nh_.param("/mission/cage_size", cage_size_, 0.10);
    nh_.param("/mission/cage_move_search", cage_move_, 0.10);

    nh_.param("/mission/turn_left", turn_left_, false);
    // 2. Kierunki osi klatki (jednostkowe)
    geometry_msgs::Point vecRight, vecUp;
    if (wall_start_ == "bottom") {
        vecRight = makePoint(0, (turn_left_ ? -1 : 1)); // oś W na Y
        vecUp = makePoint(1, 0); // oś H na X
    } else if (wall_start_ == "top") {
        vecRight = makePoint(0, (turn_left_ ? 1 : -1));
        vecUp = makePoint(-1, 0);
    } else if (wall_start_ == "left") {
        vecRight = makePoint(1, 0);
        vecUp = makePoint(0, (turn_left_ ? -1 : 1));
    } else /* "right" */ {
        vecRight = makePoint(-1, 0);
        vecUp = makePoint(0, (turn_left_ ? 1 : -1));
    }

    // 3. Parametry
    const double W = cage_width_ + 2 * cage_move_;
    const double H = cage_height_ + 2 * cage_move_;

    // 4. Funkcja dodająca przesunięcie do punktu
    auto add = [](const geometry_msgs::Point &a, const geometry_msgs::Point &v, const double s) -> geometry_msgs::Point {
        geometry_msgs::Point p;
        p.x = a.x + v.x * s;
        p.y = a.y + v.y * s;
        p.z = 0;
    return p;

    };

    // 5. Przesuń cornerPoint o -cage_move w obu kierunkach, zgodnie z lokalną osią ramki:
    geometry_msgs::Point bl = add(add(cornerPoint, vecRight, -cage_move_), vecUp, -cage_move_);
    geometry_msgs::Point br = add(bl, vecRight, W);
    geometry_msgs::Point tl = add(bl, vecUp, H);
    geometry_msgs::Point tr = add(tl, vecRight, W);

    virtual_costmap_layer::Obstacles msg;
    auto addWall = [&](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
        virtual_costmap_layer::Form wall;
        wall.form = {a, b};
        // Jeśli virtual_costmap_layer::Form ma pole grubości, można dodać: wall.size = cage_size_;
        msg.list.push_back(wall);
    };
    addWall(bl, br);
    addWall(br, tr);
    addWall(tr, tl);
    addWall(tl, bl);

    wall_pub_.publish(msg);

    ROS_INFO_STREAM("[SearchOrientation] Cage made ‑ corner=("
        << bl.x << ", " << bl.y << "), size: " << W << " x " << H
        << ", cage_move=" << cage_move_ << ", cage_size=" << cage_size_);
}

