#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>

class MissionCollector {
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber start_sub_, odom_sub_, battery_sub_;
  ros::Publisher duration_pub_, length_pub_, energy_pub_;
  ros::Timer timer_;

  std::string start_topic_, odom_topic_, battery_topic_;
  std::string duration_topic_, length_topic_, energy_topic_;
  double publish_rate_;

  bool collecting_ = false;
  ros::Time start_time_;
  double mission_duration_ = 0.0;
  double mission_length_ = 0.0;
  double mission_energy_ = 0.0;
  double start_charge_ = 0.0;
  double last_charge_ = 0.0;
  bool first_battery_ = true;


  geometry_msgs::Point last_position_;
  bool first_odom_ = true;

public:
  MissionCollector() : pnh_("~") {
    pnh_.param<std::string>("start_topic", start_topic_, "/start_collection");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<std::string>("battery_topic", battery_topic_, "/battery_state");

    pnh_.param<std::string>("duration_topic", duration_topic_, "/mission_duration");
    pnh_.param<std::string>("length_topic", length_topic_, "/mission_length");
    pnh_.param<std::string>("energy_topic", energy_topic_, "/mission_energy");

    pnh_.param<double>("publish_rate", publish_rate_, 1.0);

    start_sub_ = nh_.subscribe(start_topic_, 1, &MissionCollector::startCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MissionCollector::odomCallback, this);
    battery_sub_ = nh_.subscribe(battery_topic_, 10, &MissionCollector::batteryCallback, this);

    duration_pub_ = nh_.advertise<std_msgs::Float32>(duration_topic_, 1);
    length_pub_ = nh_.advertise<std_msgs::Float32>(length_topic_, 1);
    energy_pub_ = nh_.advertise<std_msgs::Float32>(energy_topic_, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &MissionCollector::timerCallback, this);
  }

  void startCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !collecting_) {
      collecting_ = true;
      start_time_ = ros::Time::now();
      mission_duration_ = 0.0;
      mission_length_ = 0.0;
      mission_energy_ = 0.0;
      first_odom_ = true;
      first_battery_ = true;
      ROS_INFO("Mission data collection started.");
    }else if (!msg->data && collecting_) {
      collecting_ = false;
      ROS_INFO("Mission data collection stopped.");
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!collecting_) return;

    const geometry_msgs::Point& pos = msg->pose.pose.position;
    if (first_odom_) {
      last_position_ = pos;
      first_odom_ = false;
      return;
    }

    double dx = pos.x - last_position_.x;
    double dy = pos.y - last_position_.y;
    double dz = pos.z - last_position_.z;

    mission_length_ += std::sqrt(dx*dx + dy*dy + dz*dz);
    last_position_ = pos;
  }

  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    if (!collecting_) return;

    last_charge_ = msg->charge;

    if (first_battery_) {
      start_charge_ = msg->charge;
      first_battery_ = false;
    }
  }


  void timerCallback(const ros::TimerEvent&) {
    if (!collecting_) return;

    mission_duration_ = (ros::Time::now() - start_time_).toSec();

    std_msgs::Float32 duration_msg;
    duration_msg.data = mission_duration_;
    duration_pub_.publish(duration_msg);

    std_msgs::Float32 length_msg;
    length_msg.data = mission_length_;
    length_pub_.publish(length_msg);

    std_msgs::Float32 energy_msg;
    energy_msg.data = std::max(0.0, start_charge_ - last_charge_);  // unikamy ujemnych przez pomyłki
    energy_pub_.publish(energy_msg);
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_collector_node");
  MissionCollector collector;
  ros::spin();
  return 0;
}
