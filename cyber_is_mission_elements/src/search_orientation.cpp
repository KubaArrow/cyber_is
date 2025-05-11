//
// Created by victoria on 11.05.25.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CornerFinder {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber line_sub_;
  ros::Publisher cmd_vel_pub_;

  std::string cmd_vel_topic_, line_detector_topic_, wall_origin_;
  double rotate_speed_;
  bool rotate_left_;
  bool searching_ = true;
  bool goal_sent_ = false;

  MoveBaseClient move_client_;

public:
  CornerFinder() : pnh_("~"), move_client_("move_base", true) {
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<std::string>("line_detector_topic", line_detector_topic_, "/line_detector_position");
    pnh_.param<double>("rotate_speed", rotate_speed_, 0.3);
    pnh_.param<bool>("rotate_left", rotate_left_, true);
    pnh_.param<std::string>("wall_origin", wall_origin_, "bottom");


    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &CornerFinder::lineCallback, this);

    ROS_INFO("Waiting for move_base action server...");
    move_client_.waitForServer();
    ROS_INFO("CornerFinder initialized. Rotating...");

    startRotation();
  }

  void startRotation() {
    geometry_msgs::Twist twist;
    twist.angular.z = rotate_left_ ? rotate_speed_ : -rotate_speed_;
    cmd_vel_pub_.publish(twist);
  }

  void stopRotation() {
    geometry_msgs::Twist twist;
    twist.angular.z = 0.0;
    cmd_vel_pub_.publish(twist);
  }

  void lineCallback(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data;

    if (searching_ && data == "CENTER") {
      ROS_INFO("CENTER detected. Stopping rotation and sending goal.");
      searching_ = false;
      stopRotation();
      sendGoalAhead();
    }
    else if (goal_sent_ && (data.find("LEFT") != std::string::npos || data.find("RIGHT") != std::string::npos)) {
      ROS_WARN("Line shifted from CENTER -> %s. Cancelling goal.", data.c_str());
      move_client_.cancelAllGoals();
      goal_sent_ = false;
      std::string corner = determineCorner(wall_origin_, rotate_left_);
      ROS_INFO("Detected corner: %s", corner.c_str());

    }
  }

  void sendGoalAhead() {
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
      listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("TF Error: %s", ex.what());
      return;
    }

    double yaw = tf::getYaw(transform.getRotation());

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = transform.getOrigin().x() + 6.0 * cos(yaw);
    goal.target_pose.pose.position.y = transform.getOrigin().y() + 6.0 * sin(yaw);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ROS_INFO("Sending goal 6 meters ahead...");
    move_client_.sendGoal(goal);
    goal_sent_ = true;
  }

  std::string determineCorner(const std::string& origin, bool rotate_left) {
    if (origin == "top") {
      return rotate_left ? "top_left" : "top_right";
    } else if (origin == "bottom") {
      return rotate_left ? "bottom_right" : "bottom_left";
    } else if (origin == "left") {
      return rotate_left ? "bottom_left" : "top_left";
    } else if (origin == "right") {
      return rotate_left ? "top_right" : "bottom_right";
    } else {
      return "unknown";
    }
  }


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "corner_finder");
  CornerFinder node;
  ros::spin();
  return 0;
}
