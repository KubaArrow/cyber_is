//
// Created by victoria on 11.04.25.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class SearchStartController {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber line_sub_, magnet_sub_;
  ros::Publisher cmd_vel_pub_, state_pub_;

  std::string robot_state_topic_, cmd_vel_topic_, line_detector_topic_, magnet_sensor_topic_;
  double base_speed_;
  bool search_active_ = false;
  bool full_line_detected_ = false;

public:
  SearchStartController() : pnh_("") {
    pnh_.param<std::string>("robot_state_topic", robot_state_topic_, "/robot_state");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<std::string>("line_detector_topic", line_detector_topic_, "/line_detector_position");
    pnh_.param<std::string>("magnet_sensor_topic", magnet_sensor_topic_, "/magnet_filtered");
    pnh_.param<double>("base_speed", base_speed_, 0.2);

    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &SearchStartController::lineCallback, this);
    magnet_sub_ = nh_.subscribe(magnet_sensor_topic_, 1, &SearchStartController::magnetCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    state_pub_ = nh_.advertise<std_msgs::String>(robot_state_topic_, 1);

    search_active_ = true;  // Od razu aktywujemy wyszukiwanie
    full_line_detected_ = false;

    ROS_INFO("SearchStartController initialized. Moving forward.");
    publishSpeed(base_speed_);
  }

  void lineCallback(const std_msgs::String::ConstPtr& msg) {
    if (!search_active_) return;

    if (msg->data == "FULL_LINE" && !full_line_detected_) {
      full_line_detected_ = true;
      ROS_INFO("FULL_LINE detected. Slowing down and enabling magnet check.");
      publishSpeed(base_speed_ * 0.5);
    }
  }

  void magnetCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!search_active_ || !full_line_detected_) return;

    if (msg->data) {
      ROS_INFO("Magnet detected. Stopping and publishing START_SEARCHED.");
      publishSpeed(0.0);

      std_msgs::String state_msg;
      state_msg.data = "START_SEARCHED";
      state_pub_.publish(state_msg);

      search_active_ = false;
    }
  }

  void publishSpeed(double speed) {
    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    twist.angular.z = 0.0;  // dla pewności
    ros::Duration(0.5).sleep();
    cmd_vel_pub_.publish(twist);
    ROS_INFO("Publishing cmd_vel: linear.x = %.2f", speed);
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_start_controller");
  SearchStartController node;
  ros::spin();
  return 0;
}
