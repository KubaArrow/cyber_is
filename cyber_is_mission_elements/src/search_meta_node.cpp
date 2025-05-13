// search_meta.cpp – v4 (May 2025)
//
//  🆕  Fixes edge‑detection frame issue & adds robust constant‑velocity publish.
//       • Pose is transformed into the zone’s frame (default "map") via tf2.
//       • Constant forward cmd_vel is streamed at 10 Hz while ALONG_LINE.
//       • Edge detection and rotations unchanged otherwise.
// ------------------------------------------------------------------------------
//  Build rules unchanged (search_meta target).

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>
#include <unordered_map>
#include <cmath>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class SearchMeta
{
public:
  SearchMeta(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), ac_("move_base", true), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    if (!readZonePoints())
    {
      ROS_FATAL("~zone_points invalid or missing (need 4 [x,y] pairs)");
      ros::shutdown();
    }

    initial_forward_   = pnh_.param("initial_forward", 1.0);
    goal_timeout_      = pnh_.param("goal_timeout", 300.0);
    frame_id_          = pnh_.param<std::string>("frame_id", "map");   // target frame for edges
    final_orientation_ = pnh_.param<std::string>("final_orientation", "none");

    orient_map_ = {{"none", std::numeric_limits<double>::quiet_NaN()},
                   {"right", 0.0}, {"top", M_PI_2}, {"bottom", -M_PI_2}, {"left", M_PI}};
    if (!orient_map_.count(final_orientation_)) {
      ROS_WARN("Unknown final_orientation '%s' – using 'none'", final_orientation_.c_str());
      final_orientation_ = "none";
    }

    line_sub_   = nh_.subscribe<std_msgs::String>("/line_detector_position", 1, &SearchMeta::lineCb, this);
    magnet_sub_ = nh_.subscribe<std_msgs::Bool>("/magnet/filtered", 1, &SearchMeta::magnetCb, this);

    state_pub_  = nh_.advertise<std_msgs::String>("/robot_state", 1, true);
    vel_pub_    = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Waiting for move_base …");
    ac_.waitForServer();
    ROS_INFO("move_base ready");

    phase_ = Phase::IDLE;
    edges_hit_ = 0;

    startMission();
  }

private:
  enum class Phase { IDLE, FORWARD_TO_LINE, ROTATE_90, ALONG_LINE, ROTATE_180, FINAL_ORIENT, FINISHED };

  ros::NodeHandle nh_, pnh_;
  MoveBaseClient  ac_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  double initial_forward_, goal_timeout_;
  std::string frame_id_;
  std::string final_orientation_;
  std::unordered_map<std::string,double> orient_map_;

  // zone bounds
  double min_x_, max_x_, min_y_, max_y_;

  // runtime
  Phase phase_;
  bool  full_line_detected_ = false;
  int   edges_hit_ = 0;
  ros::Time goal_sent_;
  geometry_msgs::Twist forward_cmd_;

  // ROS
  ros::Subscriber line_sub_, magnet_sub_;
  ros::Publisher  state_pub_, vel_pub_;

  /* ---------- zone parsing ---------------- */
  bool readZonePoints()
  {
    XmlRpc::XmlRpcValue arr;
    if (!pnh_.getParam("zone_points", arr) && !nh_.getParam("zone_points", arr))
      return false;
    if (arr.getType()!=XmlRpc::XmlRpcValue::TypeArray || arr.size()!=4) return false;

    std::vector<double> xs, ys;
    for (int i=0;i<arr.size();++i){
      if (arr[i].getType()!=XmlRpc::XmlRpcValue::TypeArray||arr[i].size()!=2)return false;
      xs.push_back(double(arr[i][0])); ys.push_back(double(arr[i][1])); }
    min_x_=*std::min_element(xs.begin(),xs.end()); max_x_=*std::max_element(xs.begin(),xs.end());
    min_y_=*std::min_element(ys.begin(),ys.end()); max_y_=*std::max_element(ys.begin(),ys.end());
    ROS_INFO("Zone bounds in %s: x[%.2f,%.2f] y[%.2f,%.2f]",frame_id_.c_str(),min_x_,max_x_,min_y_,max_y_);
    return true;
  }

  /* ---------- goals in base_footprint ------ */
  void sendRelativeGoal(double dx,double dy,double dyaw){
    move_base_msgs::MoveBaseGoal g; g.target_pose.header.frame_id="base_footprint";
    g.target_pose.header.stamp=ros::Time::now();
    g.target_pose.pose.position.x=dx; g.target_pose.pose.position.y=dy;
    tf2::Quaternion q; q.setRPY(0,0,dyaw); g.target_pose.pose.orientation=tf2::toMsg(q);
    ac_.sendGoal(g); goal_sent_=ros::Time::now(); }
  void sendForward(double m){sendRelativeGoal(m,0,0);} void sendRot(double deg){sendRelativeGoal(0,0,deg*M_PI/180.0);}

  /* ---------- helpers ---------------------- */
  void publishState(const std::string&s){std_msgs::String m; m.data=s; state_pub_.publish(m);}
  void stopRobot(){ac_.cancelAllGoals(); geometry_msgs::Twist z; vel_pub_.publish(z);}

  geometry_msgs::PoseStamped mapPose(){
    geometry_msgs::PoseStamped ps; ps.header.frame_id="base_link"; ps.header.stamp=ros::Time(0);
    ps.pose.orientation.w=1.0; // identity
    try{
      return tf_buffer_.transform(ps, frame_id_);
    }catch(const tf2::TransformException& e){ROS_WARN_THROTTLE(2.0,"TF error: %s",e.what()); return ps;}
  }

  /* ---------- callbacks -------------------- */
  void lineCb(const std_msgs::String::ConstPtr& m){
    if(!searchActive()||m->data!="FULL_LINE"||full_line_detected_)return;
    full_line_detected_=true; stopRobot(); publishState("FOUNDED_START_POSE");
    phase_=Phase::ROTATE_90; sendRot(90);
  }
  void magnetCb(const std_msgs::Bool::ConstPtr& b){ if(!b->data||phase_==Phase::FINISHED)return; stopRobot(); publishState("FOUNDED_FINISH"); phase_=Phase::FINISHED; }

  /* ---------- mission control -------------- */
  void startMission(){ phase_=Phase::FORWARD_TO_LINE; sendForward(initial_forward_); ROS_INFO("Drive %.2fm forward",initial_forward_); }

  bool searchActive()const{return phase_==Phase::FORWARD_TO_LINE||phase_==Phase::ALONG_LINE;}
  static double rad2deg(double r){return r*180.0/M_PI;}

public:
  void spin(){ ros::Rate loop(10);
    while(ros::ok()){
      ros::spinOnce();

      /* constant cmd */
      if(phase_==Phase::ALONG_LINE) vel_pub_.publish(forward_cmd_);

      /* edge detection (map frame) */
      if(phase_==Phase::ALONG_LINE){ auto pose=mapPose(); double y=pose.pose.position.y;
        bool hit=false; if(edges_hit_==0&&fabs(y-max_y_)<0.05)hit=true; else if(edges_hit_==1&&fabs(y-min_y_)<0.05)hit=true;
        if(hit){ ++edges_hit_; stopRobot(); ROS_INFO("Edge %d reached",edges_hit_);
          if(edges_hit_>=2){ phase_=Phase::FINAL_ORIENT; double yaw=orient_map_[final_orientation_]; if(std::isnan(yaw)){ publishState("ABORT_MISSSION"); phase_=Phase::FINISHED;} else sendRot(rad2deg(yaw)); }
          else{ phase_=Phase::ROTATE_180; sendRot(180);} }
      }

      /* timeouts */
      if((phase_==Phase::FORWARD_TO_LINE||phase_==Phase::ROTATE_90||phase_==Phase::ROTATE_180||phase_==Phase::FINAL_ORIENT)
          && !ac_.getState().isDone() && (ros::Time::now()-goal_sent_).toSec()>goal_timeout_){ ROS_WARN("Goal timeout"); stopRobot(); publishState("ABORT_MISSSION"); phase_=Phase::FINISHED; }

      /* goal finished transitions */
      if(ac_.getState().isDone()){
        if(phase_==Phase::ROTATE_90 && ac_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){ phase_=Phase::ALONG_LINE; forward_cmd_=geometry_msgs::Twist(); forward_cmd_.linear.x=0.15; }
        else if(phase_==Phase::ROTATE_180 && ac_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){ phase_=Phase::ALONG_LINE; forward_cmd_=geometry_msgs::Twist(); forward_cmd_.linear.x=0.15; }
        else if(phase_==Phase::FINAL_ORIENT && ac_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){ publishState("ABORT_MISSSION"); phase_=Phase::FINISHED; }
      }

      loop.sleep(); }
  }
};

int main(int argc,char** argv){ ros::init(argc,argv,"search_meta"); ros::NodeHandle nh,pnh("~"); SearchMeta sm(nh,pnh); sm.spin(); return 0; }
