// zone_navigator.cpp  — v2 with configurable final heading
// -------------------------------------------------------
// Adds parameter  ~final_orientation  (string)
//   values: "none" (default) | "top" | "right" | "bottom" | "left"
//             top    = +Y axis   (yaw  +90°  /  +π/2)
//             right  = +X axis   (yaw   0°)
//             bottom = –Y axis   (yaw  -90°  /  -π/2)
//             left   = –X axis   (yaw ±180° /   π )
// If set, robot will rotate accordingly at goal.
// -------------------------------------------------------
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>

class ZoneNavigator
{
public:
  ZoneNavigator() : ac_("move_base", true)
  {
    ros::NodeHandle nh("~");

    // ------------------- parameters -------------------
    XmlRpc::XmlRpcValue pts_param;
    if (!nh.getParam("zone_points", pts_param) || pts_param.getType() != XmlRpc::XmlRpcValue::TypeArray || pts_param.size() != 4)
    {
      ROS_ERROR("~zone_points must be an array of four [x,y] lists");
      ros::shutdown();
      return;
    }

    frame_id_      = nh.param<std::string>("frame_id", "map");
    goal_timeout_  = nh.param<double>("goal_timeout", 300.0);
    orientation_id_ = nh.param<std::string>("final_orientation", "none"); // new param

    // validate orientation keyword
    static const std::unordered_map<std::string, double> orient_map = {
        {"none",   std::numeric_limits<double>::quiet_NaN()}, // keep current heading
        {"right",  0.0},
        {"top",    M_PI_2},
        {"bottom", -M_PI_2},
        {"left",   M_PI}};
    auto it = orient_map.find(orientation_id_);
    if (it == orient_map.end()) {
      ROS_WARN_STREAM("Unknown final_orientation '" << orientation_id_ << "' – falling back to 'none'");
      orientation_id_ = "none";
    }

    // convert points
    for (int i = 0; i < pts_param.size(); ++i)
    {
      XmlRpc::XmlRpcValue pt = pts_param[i];
      if (pt.getType() != XmlRpc::XmlRpcValue::TypeArray || pt.size() != 2)
      {
        ROS_ERROR("Each point must be [x, y]");
        ros::shutdown();
        return;
      }
      double x = static_cast<double>(pt[0]);
      double y = static_cast<double>(pt[1]);
      points_.push_back({x, y});
    }

    // ------------------- publishers / action client -------------------
    poly_pub_  = nh.advertise<geometry_msgs::PolygonStamped>("/zone_polygon", 1, true);
    state_pub_ = nh.advertise<std_msgs::String>("/robot_state", 1, true);

    ROS_INFO("Waiting for move_base action server...");
    if (!ac_.waitForServer(ros::Duration(20.0)))
    {
      ROS_ERROR("move_base server not available – aborting mission");
      publishAbort();
      ros::shutdown();
      return;
    }
    ROS_INFO("move_base connected");

    publishPolygon();
    sendGoalAndWait();
  }

private:
  using Point = std::pair<double, double>;
  std::vector<Point> points_;
  std::string frame_id_;
  double goal_timeout_;
  std::string orientation_id_;

  ros::Publisher poly_pub_;
  ros::Publisher state_pub_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

  // ------------------- helper methods -------------------
  void publishPolygon()
  {
    geometry_msgs::PolygonStamped poly;
    poly.header.stamp = ros::Time::now();
    poly.header.frame_id = frame_id_;
    for (const auto &p : points_)
    {
      geometry_msgs::Point32 pt; pt.x = p.first; pt.y = p.second; pt.z = 0.0;
      poly.polygon.points.push_back(pt);
    }
    poly_pub_.publish(poly);
    ROS_INFO("Zone polygon published on /zone_polygon");
  }

  std::pair<double, double> centroid() const
  {
    double sx = 0, sy = 0;
    for (const auto &p : points_) { sx += p.first; sy += p.second; }
    return {sx / points_.size(), sy / points_.size()};
  }

  // yaw in radians -> quaternion
  static geometry_msgs::Quaternion yawToQuaternion(double yaw)
  {
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    geometry_msgs::Quaternion q_msg;
    tf2::convert(q, q_msg);
    return q_msg;
  }

  void sendGoalAndWait()
  {
    auto c = centroid();
    ROS_INFO_STREAM("Sending robot to zone center (" << c.first << ", " << c.second << ") in " << frame_id_ << " with orientation '" << orientation_id_ << "'");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = c.first;
    goal.target_pose.pose.position.y = c.second;
    goal.target_pose.pose.position.z = 0.0;

    // orientation
    double yaw = std::numeric_limits<double>::quiet_NaN();
    if (orientation_id_ == "top") yaw = 0.0;
    else if (orientation_id_ == "right") yaw = -M_PI_2;
    else if (orientation_id_ == "left") yaw = M_PI_2;
    else if (orientation_id_ == "bottom") yaw = M_PI;

    if (std::isnan(yaw)) {
      goal.target_pose.pose.orientation.w = 1.0; // keep current orientation (identity)
    } else {
      goal.target_pose.pose.orientation = yawToQuaternion(yaw);
    }

    ac_.sendGoal(goal);

    bool finished = ac_.waitForResult(ros::Duration(goal_timeout_));
    if (!finished) { ROS_WARN("Navigation timed out – aborting mission"); ac_.cancelGoal(); publishAbort(); return; }

    auto state = ac_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) { ROS_INFO("Robot reached zone – success"); publishState("FOUNDED_ZONE"); }
    else { ROS_WARN_STREAM("Navigation failed: " << state.toString()); publishAbort(); }
  }

  // ------------------- state publishing -------------------
  void publishAbort()   { publishState("ABORT_MISSSION"); }
  void publishState(const std::string &msg)
  {
    std_msgs::String s; s.data = msg; state_pub_.publish(s);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zone_navigator");
  ZoneNavigator zn;
  ros::spin();
  return 0;
}
