#include <ros/ros.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "global_path_plan_node");
  ros::NodeHandle nh;

  ros::Publisher plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/DWAPlannerROS/global_plan", 1);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base/current_goal", 10);

  ros::ServiceClient nav_plan_srv_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  nav_msgs::GetPlan nav_plan_srv;

  geometry_msgs::PoseStamped start_p, goal_p;
  start_p.header.frame_id = "map";
  start_p.pose.position.x = 2.0;
  start_p.pose.position.y = 2.0;
  start_p.pose.orientation.w = 1.0;

  goal_p.header.frame_id = "map";
  goal_p.pose.position.x = 7.0;
  goal_p.pose.position.y = 4.0;
  goal_p.pose.orientation.w = 1.0;

  nav_plan_srv.request.start = start_p;
  nav_plan_srv.request.goal = goal_p;
  nav_plan_srv.request.tolerance = 0.05;

  // TODO: not display on the RViz yet
  goal_pub.publish(goal_p);

  nav_msgs::Path planned_path;
  if (nav_plan_srv_client.call(nav_plan_srv))
  {
    planned_path = nav_plan_srv.response.plan;
    ROS_INFO_STREAM("Path found, path size: " << planned_path.poses.size());
  }
  else
  {
    ROS_WARN ("Call Nav make plan service fail");
    exit(1);
  }

  plan_pub_.publish(planned_path);

//  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1);
//  double max_vel_x = 0.0;
//  double desired_vel = 0.11; // elder people 0.11-0.15
//  nh.param("/move_base/DWAPlannerROS/max_vel_x", max_vel_x, 0.3);

//  for (int i = 0; i < planned_path.poses.size()-1; i++)
//  {
//    ros::Time t_start = planned_path.poses[i].header.stamp;
//    ros::Time t_end = planned_path.poses[i+1].header.stamp;
//    ros::Duration dura = t_end-t_start;

//    float time_diff = dura.toSec();
//    geometry_msgs::Twist cmd;
//    cmd.linear.x = (planned_path.poses[i+1].pose.position.x-planned_path.poses[i].pose.position.x)/time_diff;
////    cmd.angular.z
//    cmd.linear.x = 0.05;
//    cmd_pub.publish(cmd);
//    ros::Duration(1.0).sleep();
//  }


  return 0;
}
