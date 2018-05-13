#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <turtlebot_guide_msgs/FollowerState.h>

#include <vector>

visualization_msgs::Marker getTextMarker(turtlebot_guide_msgs::FollowerState state);

void stateCB(const turtlebot_guide_msgs::FollowerStateConstPtr &_msg);

std::vector<geometry_msgs::Point32> getFollowingPoints();

std::vector<geometry_msgs::Point32> getNonFollowingPoints();

std::vector<geometry_msgs::Point32> getPersonalSpacePoints();

std::vector<geometry_msgs::Point32> getSocialSpacePoints();

std::vector<geometry_msgs::Point32> getCirclePoints(const float &radius, const float &orig_x, const float &orig_y);

const std::string base_link_ = "base_footprint";
turtlebot_guide_msgs::FollowerState m_state;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;

  ros::Publisher polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("turtlebot_guide/visualization", 10);
  ros::Publisher text_pub = nh.advertise<visualization_msgs::Marker>("follower_state_marker", 1);
  ros::Subscriber monitor_sub = nh.subscribe("follower_state", 10, stateCB);

  geometry_msgs::PolygonStamped polygon_following;
  polygon_following.header.frame_id = "base_footprint";
  polygon_following.header.stamp = ros::Time::now();
  polygon_following.polygon.points = getFollowingPoints();

  geometry_msgs::PolygonStamped polygon_nonfollowing;
  polygon_nonfollowing.header.frame_id = "base_footprint";
  polygon_nonfollowing.header.stamp = ros::Time::now();
  polygon_nonfollowing.polygon.points = getNonFollowingPoints();

  unsigned int label = 0;
  jsk_recognition_msgs::PolygonArray polygon_array;
  polygon_array.header.frame_id = "base_footprint";
  polygon_array.header.stamp = ros::Time::now();
  polygon_array.polygons.push_back(polygon_following);
  polygon_array.likelihood.push_back(1.0);
  polygon_array.labels.push_back(label);
  label += 10;

  polygon_array.polygons.push_back(polygon_nonfollowing);
  polygon_array.likelihood.push_back(1.0);
  polygon_array.labels.push_back(label);
  label += 10;

//  geometry_msgs::PolygonStamped polygon_personal;
//  polygon_personal.header.frame_id = "base_footprint";
//  polygon_personal.header.stamp = ros::Time::now();
//  polygon_personal.polygon.points = getPersonalSpacePoints();

//  polygon_array.polygons.push_back(polygon_personal);
//  polygon_array.likelihood.push_back(0.5);
//  polygon_array.labels.push_back(label);
//  label += 10;

//  geometry_msgs::PolygonStamped polygon_social;
//  polygon_social.header.frame_id = "base_footprint";
//  polygon_social.header.stamp = ros::Time::now();
//  polygon_social.polygon.points = getSocialSpacePoints();

//  polygon_array.polygons.push_back(polygon_social);
//  polygon_array.likelihood.push_back(1.0);
//  polygon_array.labels.push_back(label);
//  label += 10;

  while(polygon_pub.getNumSubscribers() == 0 || text_pub.getNumSubscribers() == 0){
    ros::Duration(0.05).sleep();
  }

  ros::Rate r(100);
  while(ros::ok()){
    visualization_msgs::Marker text_marker_msg = getTextMarker(m_state);
    text_pub.publish(text_marker_msg);

    polygon_pub.publish(polygon_array);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main

visualization_msgs::Marker getTextMarker(turtlebot_guide_msgs::FollowerState state)
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = base_link_;
  text_marker.header.stamp = ros::Time::now();
  text_marker.id = 100;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.scale.z = 0.5;

  switch (state.state)
  {
    case turtlebot_guide_msgs::FollowerState::FOLLOWING:
      text_marker.text = "Following";
      break;
    case turtlebot_guide_msgs::FollowerState::NOT_FOLLOWING:
      text_marker.text = "Not_Following";
      break;
    case turtlebot_guide_msgs::FollowerState::STOPPED:
      text_marker.text = "Stopped";
      break;
    case turtlebot_guide_msgs::FollowerState::UNKNOWN:
      text_marker.text = "Unknown";
      break;
  }

  return text_marker;
}

void stateCB(const turtlebot_guide_msgs::FollowerStateConstPtr &_msg)
{
  m_state = *_msg;
}

std::vector<geometry_msgs::Point32> getFollowingPoints()
{
  std::vector<geometry_msgs::Point32> points;
  geometry_msgs::Point32 origin;
  origin.x = -0.6314-0.11; origin.y = 0.2944; origin.z = 0.0;
  points.push_back(origin);

  geometry_msgs::Point32 via1, via2, via3;
  via1.x = -3.61; via1.y = 2.084-0.09; via1.z = 0.0;
  points.push_back(via1);

  via2.x = -3.61; via2.y = -2.084; via2.z = 0.0;
  points.push_back(via2);

  via3.x = -0.6314-0.11; via3.y = -0.2944-0.1; via3.z = 0.0;
  points.push_back(via3);
  points.push_back(origin);
  return points;
}

std::vector<geometry_msgs::Point32> getNonFollowingPoints()
{
  std::vector<geometry_msgs::Point32> points;
  geometry_msgs::Point32 origin;
  origin.x = -0.7314-0.11; origin.y = 0.2944; origin.z = 0.0;
  points.push_back(origin);

  geometry_msgs::Point32 via1, via2, via3;
  via1.x = -2.8; via1.y = 1.3-0.09; via1.z = 0.0;
  points.push_back(via1);

  via2.x = -2.8; via2.y = -1.3; via2.z = 0.0;
  points.push_back(via2);

  via3.x = -0.7314-0.11; via3.y = -0.2944-0.1; via3.z = 0.0;
  points.push_back(via3);
  points.push_back(origin);
  return points;
}

std::vector<geometry_msgs::Point32> getPersonalSpacePoints()
{
  std::vector<geometry_msgs::Point32> points = getCirclePoints(1.2, 1.0, 0.0);
  return points;
}

std::vector<geometry_msgs::Point32> getSocialSpacePoints()
{
  std::vector<geometry_msgs::Point32> points = getCirclePoints(3.6, 0.0, 0.0);
  return points;
}

std::vector<geometry_msgs::Point32> getCirclePoints(const float &radius, const float &orig_x, const float &orig_y)
{
  std::vector<geometry_msgs::Point32> points;
  float theta = 0.0;
  for(int i = 0; i < 10; i++){
    geometry_msgs::Point32 p;
    float coord_x = radius*std::cos(theta*M_PI/180.0);
    float coord_y = radius*std::sin(theta*M_PI/180.0);
    p.x = coord_x;
    p.y = coord_y;
    points.push_back(p);
    ROS_INFO_STREAM(std::cos(theta*M_PI/180.0) << " " << radius*std::sin(theta*M_PI/180.0));
    theta += 30.0;
  }
}
