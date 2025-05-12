// move_and_build_node.cpp — ROS Noetic node

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <virtual_costmap_layer/Obstacles.h>
#include <cmath>

class MoveAndBuildNode
{
  ros::NodeHandle nh_{"~"};
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client_{"move_base", true};
  ros::Subscriber line_sub_, odom_sub_;
  ros::Publisher  state_pub_;
  ros::Publisher  wall_pub_;
  ros::Publisher  cmd_vel_pub_;
  std::string robot_state_topic_, line_detector_topic_, wall_topic_, odom_topic_, wall_start_;

  double move_front_ = 1.5;   // [m]
  double move_side_ = 1.0;   // [m]
  double cage_height_ = 1.0;   // [m]
  double cage_width_ = 1.0;   // [m]
  double cage_size_ = 1.0;   // [m]
  double cage_move_ = 1.0;   // [m]
  bool turn_left_  = true;

  geometry_msgs::Point cornerPoint;
  geometry_msgs::Pose current_pose_;

  bool search_active_      = false;   // start true only for the Y‑segment
  bool full_line_detected_ = false;   // latched on first FULL_LINE

public:
  MoveAndBuildNode()
  {
    // ───────── parameters ─────────
    nh_.param<std::string>("robot_state_topic", robot_state_topic_, "/robot_state");
    nh_.param<std::string>("line_detector_topic", line_detector_topic_, "/line_detector_position");
    nh_.param<std::string>("wall_topic", wall_topic_, "/virtual_obstacles");
    nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>("wall_start", wall_start_, "bottom");
    nh_.param("move_front", move_front_, 1.5);
    nh_.param("move_side", move_side_, 1.0);
    nh_.param("cage_height", cage_height_, 2.50);
    nh_.param("cage_width",  cage_width_,  2.50);
    nh_.param("cage_size",   cage_size_,   0.10);
    nh_.param("cage_move",   cage_move_,   0.10);

    nh_.param("turn_left",  turn_left_,  false);


    // ───────── topics ─────────
    line_sub_   = nh_.subscribe(line_detector_topic_, 1, &MoveAndBuildNode::lineCallback, this);
    state_pub_  = nh_.advertise<std_msgs::String>(robot_state_topic_, 1, true);
    wall_pub_   = nh_.advertise<virtual_costmap_layer::Obstacles>(wall_topic_, 1, true);
    cmd_vel_pub_= nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MoveAndBuildNode::odomCallback, this);


    ROS_INFO("[move_and_build_node] Waiting for move_base action server …");
    mb_client_.waitForServer();
    ROS_INFO("[move_and_build_node] Connected to move_base, starting mission");

    executeSequence();
  }

private:
  // ————————————————————————————————————————— mission sequence ——————————————————————————————————————————
  void executeSequence(){

    const double yaw = (turn_left_ ? M_PI / 2.0 : -M_PI / 2.0);
    sendRelativeGoal(move_front_, 0.0, yaw);
    if (!waitForResult()) return;

    search_active_ = true;
    sendRelativeGoal(move_side_, 0.0, 0.0);
    if (!waitForResult()) return;
    search_active_ = false;
  }

  // ————————————————————————————————————————— helpers ——————————————————————————————————————————————
  void sendRelativeGoal(double dx, double dy, double dyaw)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_footprint";   // relative frame
    goal.target_pose.header.stamp    = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dyaw);
    goal.target_pose.pose.orientation = tf2::toMsg(q);

    mb_client_.sendGoal(goal);
  }

  bool waitForResult()
  {
    // poll until goal done or FULL_LINE detected (which cancels the goal)
    ros::Rate r(20);
    while (ros::ok())
    {
      if (full_line_detected_)
      {
        ROS_WARN("[move_and_build_node] FULL_LINE detected → cancel current goal");

        return false;
      }

      if (mb_client_.getState().isDone())
        return mb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;

      ros::spinOnce();
      r.sleep();
    }
    return false;
  }


  // ————————————————————————————————————————— callbacks ————————————————————————————————————————————
  void lineCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (!search_active_) return;     // ignore until Y‑segment is active

    if (msg->data == "FULL_LINE")
    {
      mb_client_.cancelAllGoals();

      ROS_INFO("[move_and_build_node] FULL_LINE detected → stop & build virtual wall");
      odom_sub_.shutdown();
      sendRelativeGoal(-move_side_/2, 0.0, 0.0);
      if (!waitForResult()) return;
      make_walls();

      std_msgs::String st;
      st.data = "FOUNDED_ORIENTATION";
      state_pub_.publish(st);
    }
  }

  static geometry_msgs::Point makePoint(double x, double y, double z = 0.0)
  {
    geometry_msgs::Point p;
    p.x = x;  p.y = y;  p.z = z;
    return p;
  }

  // ————————————————————————————————————————— virtual wall ——————————————————————————————————————————
// ─── NOWA WERSJA FUNKCJI make_walls() ───────────────────────────────────────────
void make_walls()
{
  /* 1. start / orient  ─────────────────────────────────────────────────────── */
  geometry_msgs::Point start, orient;
  nh_.getParam("/start_pose/x",       start.x);
  nh_.getParam("/start_pose/y",       start.y);
  nh_.getParam("/orientation_pose/x", orient.x);
  nh_.getParam("/orientation_pose/y", orient.y);

  /* 2. narożnik  (bez odwracania znaków!)                                     */
  if (wall_start_ == "bottom" || wall_start_ == "top")
  {
    cornerPoint.x = start.x;
    cornerPoint.y = orient.y;

  }
  else
  {
    cornerPoint.x = orient.x;
    cornerPoint.y = start.y;
  }

  /* 3. kierunki „w prawo” i „w górę” w ramach klatki ───────────────────────── */
  geometry_msgs::Point vecRight, vecUp;   // jednostkowe wektory osi W i H

  if      (wall_start_ == "bottom")
  {
    vecRight = makePoint(0,   (turn_left_ ?  -1 : 1));   // oś W na Y
    vecUp    = makePoint(1,   0);                        // oś H na X (DO GÓRY!)
  }
  else if (wall_start_ == "top")
  {
    vecRight = makePoint(0,   (turn_left_ ? 1 :  -1));
    vecUp    = makePoint(-1,  0);
  }
  else if (wall_start_ == "left")
  {
    vecRight = makePoint(1,   0);                        // W na X
    vecUp    = makePoint(0,   (turn_left_ ? -1 :  1));   // H na Y
  }
  else /* "right" */
  {
    vecRight = makePoint(-1,  0);
    vecUp    = makePoint(0,   (turn_left_ ?  1 : -1));
  }

  const double W = cage_width_;
  const double H = cage_height_;

    auto add = [](const geometry_msgs::Point& a,
                  const geometry_msgs::Point& v,
                  double s) -> geometry_msgs::Point
    {
      return makePoint(a.x + v.x * s,
                       a.y + v.y * s);
    };


  geometry_msgs::Point bl = cornerPoint;
  geometry_msgs::Point br = add(bl, vecRight, W);
  geometry_msgs::Point tl = add(bl, vecUp,    H);
  geometry_msgs::Point tr = add(tl, vecRight, W);

  /* 4. publikacja ‑ 4 ścian jako linii ─────────────────────────────────────── */
  virtual_costmap_layer::Obstacles msg;
  auto addWall = [&](const geometry_msgs::Point& a,
                     const geometry_msgs::Point& b)
  {
    virtual_costmap_layer::Form wall;  wall.form = {a, b};  msg.list.push_back(wall);
  };
  addWall(bl, br);  addWall(br, tr);  addWall(tr, tl);  addWall(tl, bl);

  wall_pub_.publish(msg);

  ROS_INFO_STREAM("[move_and_build_node] Klatka narysowana ‑ corner=("
                  << cornerPoint.x << ", " << cornerPoint.y << ")");
}

// ───────────────────────────────────────────────────────────────────────────────


  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose_ = msg->pose.pose;
    nh_.setParam("/orientation_pose/x", current_pose_.position.x);
    nh_.setParam("/orientation_pose/y", current_pose_.position.y);
  }
};

// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────———
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_and_build_node");
  MoveAndBuildNode node;          // ctor starts the sequence
  ros::spin();                    // keep processing callbacks after mission end
  return 0;
}