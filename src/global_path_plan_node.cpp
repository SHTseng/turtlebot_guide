#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "global_path_plan_node");
  ros::NodeHandle nh;

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

  if (nav_plan_srv_client.call(nav_plan_srv))
  {
    nav_msgs::Path planned_path = nav_plan_srv.response.plan;
    ROS_INFO_STREAM("Path found, path size: " << planned_path.poses.size());
  }
  else
  {
    ROS_WARN ("Call Nav make plan service fail");
  }

  return 0;
}
