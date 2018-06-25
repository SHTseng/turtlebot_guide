#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <vector>
#include <fstream>
#include <sstream>

#include <tinyxml2.h>

using namespace tinyxml2;

double velocity = 0.35;

void writeModelFileXML(const std::vector<geometry_msgs::PoseStamped> path)
{
  if(path.size() == 0){
    ROS_INFO("path is empty");
    return;
  }

  boost::gregorian::date dayte(boost::gregorian::day_clock::local_day());
  boost::posix_time::ptime midnight(dayte);
  boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration td = now - midnight;

  std::stringstream ss;
  ss << dayte.year() << "-" << dayte.month().as_number()
       << "-" << dayte.day() << "-";
  ss << td.hours() << "-" << td.minutes() << "-" << td.seconds();

  std::string model_path = ros::package::getPath("turtlebot_guide_integration")+"/models/";
  std::string model_name = model_path + "people_" + ss.str() + ".sdf";

  double time_stamps = 0.0;
  geometry_msgs::PoseStamped pose_prev = path.front();
  XMLDocument model_file("1.0");
  XMLElement *pRoot = model_file.NewElement("trajectory");
  pRoot->SetAttribute("id", 0);
  pRoot->SetAttribute("type", "walking");
  model_file.InsertFirstChild(pRoot);

  for(auto it = path.begin(); it != path.end(); it++){
    XMLElement *pWayPt = model_file.NewElement("waypoint");
    XMLElement * pElement = model_file.NewElement("time");
    pElement->SetText(time_stamps);
    pWayPt->InsertEndChild(pElement);

    double dx = it->pose.position.x-pose_prev.pose.position.x;
    double dy = it->pose.position.y-pose_prev.pose.position.y;
    double yaw = std::atan2(dy, dx);
    time_stamps += hypot(dx, dy)/velocity;

    std::stringstream pose_stream;
    pose_stream << it->pose.position.x << " " << it->pose.position.y << " " << it->pose.position.z << " ";
    pose_stream << 0 << " " << 0 << " " << yaw;
    pElement = model_file.NewElement("pose");
    pElement->SetText(pose_stream.str().c_str());
    pWayPt->InsertEndChild(pElement);

    pRoot->InsertEndChild(pWayPt);
    pose_prev = *it;
  }

  model_file.SaveFile(model_name.c_str());
}

void writeModelFileTXT(const std::vector<geometry_msgs::PoseStamped> path)
{
  if(path.size() == 0){
    ROS_INFO("path is empty");
    return;
  }

  boost::gregorian::date dayte(boost::gregorian::day_clock::local_day());
  boost::posix_time::ptime midnight(dayte);
  boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration td = now - midnight;

  std::stringstream ss;
  ss << dayte.year() << "-" << dayte.month().as_number()
       << "-" << dayte.day() << "-";
  ss << td.hours() << "-" << td.minutes() << "-" << td.seconds();

  std::string file_path = ros::package::getPath("turtlebot_guide_integration")+"/models/";
  std::string file_name = file_path + "people_" + ss.str() + ".txt";

  std::ofstream trajectory_file;
  trajectory_file.open(file_name);
  if(!trajectory_file){
    ROS_ERROR("Unable to write file");
    return;
  }

  for(auto i : path){
    trajectory_file << i.pose.position.x << " " << i.pose.position.y;
    trajectory_file << "\n";
  }
  trajectory_file.close();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "global_path_plan_node");
  ros::NodeHandle nh;

  float goal_x = 8.0;
  float goal_y = 17.0;
  if (argc == 3)
  {
    goal_x = std::atof(argv[1]);
    goal_y = std::atof(argv[2]);
  }


  ros::ServiceClient nav_plan_srv_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base_legacy_relay/make_plan");

  geometry_msgs::PoseStamped start_p, goal_p;
  start_p.header.frame_id = "map";
  start_p.pose.position.x = 22.0; // 24 for follower, 22 for robot
  start_p.pose.position.y = 17.0;
  start_p.pose.orientation.w = 1.0;

  goal_p.header.frame_id = "map";
  goal_p.pose.position.x = goal_x;
  goal_p.pose.position.y = goal_y;
//  goal_p.pose.position.x = 13.0;
//  goal_p.pose.position.y = 8.0;
  goal_p.pose.orientation.w = 1.0;

  nav_msgs::GetPlan nav_plan_srv;
  nav_plan_srv.request.start = start_p;
  nav_plan_srv.request.goal = goal_p;
  nav_plan_srv.request.tolerance = 0.05;

  nav_msgs::Path planned_path;
  nav_plan_srv_client.waitForExistence(ros::Duration(5.0));
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

  writeModelFileTXT(planned_path.poses);

  return 0;
}
