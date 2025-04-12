//
// Created by victoria on 12.04.25.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FinishLineSeeker {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber line_sub_, magnet_sub_;
  ros::Publisher cmd_vel_pub_, state_pub_;

  std::string cmd_vel_topic_, line_detector_topic_, magnet_topic_, robot_state_topic_;
  double base_speed_;
  bool turn_left_;
  bool full_line_detected_ = false;
  bool moved_forward_ = false;
  bool turned_ = false;
  bool following_line_ = false;
  bool reversing_ = false;

  MoveBaseClient move_client_;
  tf::TransformListener tf_listener_;

public:
  FinishLineSeeker() : pnh_("~"), move_client_("move_base", true) {
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<std::string>("line_detector_topic", line_detector_topic_, "/line_detector_position");
    pnh_.param<std::string>("magnet_sensor_topic", magnet_topic_, "/magnet_filtered");
    pnh_.param<std::string>("robot_state_topic", robot_state_topic_, "/robot_state");
    pnh_.param<double>("base_speed", base_speed_, 0.2);
    pnh_.param<bool>("turn_left", turn_left_, true);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    state_pub_ = nh_.advertise<std_msgs::String>(robot_state_topic_, 1);

    line_sub_ = nh_.subscribe(line_detector_topic_, 1, &FinishLineSeeker::lineCallback, this);
    magnet_sub_ = nh_.subscribe(magnet_topic_, 1, &FinishLineSeeker::magnetCallback, this);

    move_client_.waitForServer();
    ROS_INFO("FinishLineSeeker initialized. Driving forward...");
    drive(base_speed_);
  }

  void drive(double speed) {
    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    twist.angular.z = 0.0;
    cmd_vel_pub_.publish(twist);
  }

  void rotate90() {
    geometry_msgs::Twist twist;
    twist.angular.z = turn_left_ ? 0.6 : -0.6;
    ros::Duration(1.57 / std::abs(twist.angular.z)).sleep();  // ~90 deg
    twist.angular.z = 0.0;
    cmd_vel_pub_.publish(twist);
    ros::Duration(0.5).sleep();
  }

  void lineCallback(const std_msgs::String::ConstPtr& msg) {
    if (reversing_ || !msg) return;

    std::string val = msg->data;

    if (!full_line_detected_ && val == "FULL_LINE") {
      full_line_detected_ = true;
      ROS_INFO("FULL_LINE detected. Stopping and sending small forward goal...");
      drive(0.0);
      sendSmallForwardGoal();
      return;
    }

    if (turned_ && val != "CENTER") {
      ROS_WARN("Corner detected again. Reversing...");
      reversing_ = true;
      drive(-base_speed_);
    }
  }

  void magnetCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!turned_ || !msg->data || reversing_) return;

    ROS_INFO("Magnet detected. Mission complete!");
    drive(0.0);
    std_msgs::String done;
    done.data = "MISSION_SUCCESS";
    state_pub_.publish(done);
  }

  void sendSmallForwardGoal() {
    tf::StampedTransform tf_map_base;
    try {
      tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
      tf_listener_.lookupTransform("map", "base_link", ros::Time(0), tf_map_base);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("TF Error: %s", ex.what());
      return;
    }

    double yaw = tf::getYaw(tf_map_base.getRotation());

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = tf_map_base.getOrigin().x() + 0.1 * cos(yaw);
    goal.target_pose.pose.position.y = tf_map_base.getOrigin().y() + 0.1 * sin(yaw);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    move_client_.sendGoal(goal);
    move_client_.waitForResult();

    if (move_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Moved 0.1m forward. Now rotating...");
      moved_forward_ = true;
      rotate90();
      turned_ = true;
      following_line_ = true;
      drive(base_speed_);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "finish_line_seeker");
  FinishLineSeeker seeker;
  ros::spin();
  return 0;
}
