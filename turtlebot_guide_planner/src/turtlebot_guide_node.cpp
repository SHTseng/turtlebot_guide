#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

namespace turtlebot_guide
{

class TurtlebotGuidance
{

public:

  TurtlebotGuidance():
    frame_id_("map"),
    tolerance_(0.01){
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("init_x", init_x, 0.0);
    private_nh.param("init_y", init_y, 0.0);
    private_nh.param("init_yaw", init_yaw, 0.0);
    private_nh.param("goal_x", goal_x, 0.0);
    private_nh.param("goal_y", goal_y, 0.0);
    private_nh.param("goal_yaw", goal_yaw, 0.0);
    private_nh.param("tolerance", tolerance_, 0.01);

    // Call global path planner to obtain a global path
    ros::ServiceClient nav_plan_srv_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan nav_plan_srv;
    goal_pose_ = getPoseStamped(goal_x, goal_y, goal_yaw);
    start_pose_ = getPoseStamped(init_x, init_y, init_yaw);
    nav_plan_srv.request.start = start_pose_;
    nav_plan_srv.request.goal = goal_pose_;
    nav_plan_srv.request.tolerance = tolerance_;

    ros::service::waitForService("/move_base/make_plan");
    if (nav_plan_srv_client.call(nav_plan_srv)){
      if (!global_path_.empty()) global_path_.clear();
      global_path_ = nav_plan_srv.response.plan.poses;
      ROS_INFO_STREAM("Path found, path size: " << global_path_.size());
    }
    else{
      ROS_ERROR_STREAM("Call Nav make plan service fail");
      exit(1);
    }

    odom_sub_ = nh.subscribe("odom", 1, &TurtlebotGuidance::odomCB, this);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Star local planner loop
    last_time = ros::Time::now();
    ros::Rate r(control_frequency_);
    while (ros::ok() && !global_path_.empty()){
      ros::spinOnce();
      current_time = ros::Time::now();
      geometry_msgs::PoseStamped current_goal = global_path_.front();
      global_path_.front() = std::move(global_path_.back());
      global_path_.pop_back();

      if (isGoalReached(current_goal)){

      }
      geometry_msgs::Twist cmd;
//      cmd.linear.
      vel_pub_.publish(cmd);
      r.sleep();
    }
  }

  ~TurtlebotGuidance(){
    vel_pub_.shutdown();
  }
  
private:

  void odomCB(const nav_msgs::Odometry& msg){

  }

  bool isGoalReached(const geometry_msgs::PoseStamped& pos){
    return distance(pos, goal_pose_) < tolerance_;
  }

  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    return hypot(p1.pose.position.x-p2.pose.position.x, p1.pose.position.y-p2.pose.position.y);
  }

  geometry_msgs::PoseStamped getPoseStamped(const double& x,
                                            const double& y,
                                            const double& yaw){
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.orientation = quat;
    return ps;
  }

  ros::Publisher vel_pub_;
  ros::Subscriber fpose_sub_;
  ros::Subscriber odom_sub_;

  ros::Time current_time;
  ros::Time last_time;

  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  std::vector<geometry_msgs::PoseStamped> global_path_;


  double init_x, init_y, init_yaw,
         goal_x, goal_y, goal_yaw,
         tolerance_;
  double control_frequency_;

  std::string robot_namespace_;
  std::string frame_id_;
};
}

int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_guide_node");
  turtlebot_guide::TurtlebotGuidance tg;

  //ros::spin();
  return 0;
}
