// SearchMeta.h
#ifndef SEARCHMETA_H
#define SEARCHMETA_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <utility>
#include <vector>
#include <string>

class SearchMeta {
public:
    explicit SearchMeta(ros::NodeHandle &nh,
                        const std::string &state_topic,
                        const std::string &line_detector_topic,
                        const std::string &odom_topic,
                        const std::string &wall_topic);

private:
    using Point = std::pair<double, double>;
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    // params & topics
    std::string state_topic_, line_detector_topic_, magnet_topic_, odom_topic_, wall_topic_;
    std::string final_orientation_, mode_;
    double move_cage_;

    // geometry
    std::vector<Point> points_;         // zone
    std::vector<Point> samples_points_; // sampling

    // ROS pubs/subs
    ros::Subscriber line_sub_, magnet_sub_, odom_sub_;
    ros::Publisher state_pub_, wall_pub_, vel_pub_;

    // robot state
    geometry_msgs::Pose current_pose_;
    bool sampling_active_{false};
    bool search_active_{false};
    bool full_line_detected_{false};

    // callbacks
    void lineCallback(const std_msgs::String::ConstPtr &msg);
    void magnetCallback(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    // sequences
    void executeLineSequence();
    void executeSamplingSequence();

    // helpers
    void sendRelativeGoal(double dx, double dy, double dyaw);
    void sendGoal(double x, double y, double yaw);
    bool waitForResult(double timeout_sec=60.0);
    geometry_msgs::Point makePoint(double x, double y);
    size_t findNearestSample(const geometry_msgs::Pose &pose, const std::vector<bool> &visited) const;
    double yawFromFinalOrientation() const;
    void stopRobot();
    void reverseToStart(const geometry_msgs::Pose &start, double speed);
    void publishAbort() const;
    void publishState(const std::string &msg) const;
};

#endif // SEARCHMETA_H