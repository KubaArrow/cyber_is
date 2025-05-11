//
// Created by victoria on 11.04.25.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <virtual_costmap_layer/Obstacles.h>
#include <virtual_costmap_layer/Form.h>

class SearchStartController {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber line_sub_;
  ros::Publisher cmd_vel_pub_, state_pub_, wall_pub_;

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
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    state_pub_ = nh_.advertise<std_msgs::String>(robot_state_topic_, 1);
    wall_pub_=  nh_.advertise<virtual_costmap_layer::Obstacles>("/virtual_obstacles",1, true);
    search_active_ = true;  // Od razu aktywujemy wyszukiwanie
    full_line_detected_ = false;

    ROS_INFO("SearchStartController initialized. Moving forward.");
    std_msgs::String msg;
    msg.data = "START_SEARCH_START_POSE";
    state_pub_.publish(msg);
    publishSpeed(base_speed_);
  }

  void lineCallback(const std_msgs::String::ConstPtr& msg) {
    if (!search_active_) return;

    if (msg->data == "FULL_LINE" && !full_line_detected_) {
      full_line_detected_ = true;
      ROS_INFO("FULL_LINE detected. Slowing down and enabling magnet check.");
      publishSpeed(0.0);
      std_msgs::String msg;
      msg.data = "FOUNDED_START_POSE";
      make_wall();
      state_pub_.publish(msg);
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

  void make_wall()
  {

    geometry_msgs::Point p0, p1, p2, p3;
    p0.x = -0.10; p0.y = -10.0;
    p1.x = -0.10; p1.y = 10.0;
    p2.x = -0.20; p2.y = 10.0;
    p3.x = -0.20; p3.y = -10.0;

    virtual_costmap_layer::Form polygon;
    polygon.form = {p0, p1, p2, p3};      // ← wektor punktów

    virtual_costmap_layer::Obstacles msg;
    msg.list.push_back(polygon);          // ← list to vector<Form>

    wall_pub_.publish(msg);
  }


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_start_controller");
  SearchStartController node;
  ros::spin();
  return 0;
}
