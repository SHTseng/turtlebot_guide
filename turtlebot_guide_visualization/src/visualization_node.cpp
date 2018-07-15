#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <turtlebot_guide_msgs/FollowerState.h>

#include <vector>

visualization_msgs::Marker getTextMarker(turtlebot_guide_msgs::FollowerState state);

visualization_msgs::Marker getPoseMarker(const geometry_msgs::Pose2D &state);

void stateCB(const turtlebot_guide_msgs::FollowerStateConstPtr &_msg);

std::vector<geometry_msgs::Point32> getFollowingPoints();

std::vector<geometry_msgs::Point32> getNonFollowingPoints();

std::vector<geometry_msgs::Point32> getPersonalSpacePoints();

std::vector<geometry_msgs::Point32> getSocialSpacePoints();

std::vector<geometry_msgs::Point32> getCirclePoints(const float &radius, const float &orig_x, const float &orig_y);

const std::string base_link_ = "base_footprint";
turtlebot_guide_msgs::FollowerState m_state;
std::vector<geometry_msgs::Point32> following_polygon_;
std::vector<geometry_msgs::Point32> camera_polygon_;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;

  std::vector<double> following_range;
  nh.param("/follower_state_monitor/following_range", following_range, std::vector<double>());
  for(std::size_t i = 0; i < following_range.size(); i+=2)
  {
    geometry_msgs::Point32 p;
    p.x = static_cast<float>(following_range[i]);
    p.y = static_cast<float>(following_range[i+1]);
    p.z = 0.0;
    following_polygon_.push_back(p);
  }

  std::vector<double> camera_frustum;
  nh.param("/follower_state_monitor/camera_frustum", camera_frustum, std::vector<double>());
  for(std::size_t i = 0; i < camera_frustum.size(); i+=2)
  {
    geometry_msgs::Point32 p;
    p.x = static_cast<float>(camera_frustum[i]);
    p.y = static_cast<float>(camera_frustum[i+1]);
    p.z = 0.0;
    camera_polygon_.push_back(p);
  }


  ros::Publisher polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("turtlebot_guide/visualization", 10);
  ros::Publisher pose_marker_pub = nh.advertise<visualization_msgs::Marker>("current_pose", 10);
  ros::Publisher predict_pose_marker_pub = nh.advertise<visualization_msgs::Marker>("predict_pose", 10);
  ros::Publisher text_pub = nh.advertise<visualization_msgs::Marker>("follower_state_marker", 1);
  ros::Subscriber monitor_sub = nh.subscribe("follower_state", 10, stateCB);

  geometry_msgs::PolygonStamped polygon_following;
  polygon_following.header.frame_id = "base_footprint";
  polygon_following.header.stamp = ros::Time::now();
  polygon_following.polygon.points = following_polygon_;

  geometry_msgs::PolygonStamped polygon_camera_msg;
  polygon_camera_msg.header.frame_id = "base_footprint";
  polygon_camera_msg.header.stamp = ros::Time::now();
  polygon_camera_msg.polygon.points = camera_polygon_;

  unsigned int label = 0;
  jsk_recognition_msgs::PolygonArray polygon_array;
  polygon_array.header.frame_id = "base_footprint";
  polygon_array.header.stamp = ros::Time::now();
  polygon_array.polygons.push_back(polygon_following);
  polygon_array.likelihood.push_back(1.0);
  polygon_array.labels.push_back(label);
  label += 10;

  polygon_array.polygons.push_back(polygon_camera_msg); // polygon_nonfollowing
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

  while(polygon_pub.getNumSubscribers() == 0 && text_pub.getNumSubscribers() == 0){
    ros::Duration(0.05).sleep();
  }

  ros::Rate r(100);
  while(ros::ok()){
    visualization_msgs::Marker text_marker_msg = getTextMarker(m_state);
    text_pub.publish(text_marker_msg);

//    visualization_msgs::Marker pose_marker_msgs = getPoseMarker(m_state.pose);
//    pose_marker_pub.publish(pose_marker_msgs);
    visualization_msgs::Marker predict_pose_marker_msgs = getPoseMarker(m_state.predict_pose);
    predict_pose_marker_pub.publish(predict_pose_marker_msgs);

    polygon_pub.publish(polygon_array);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main

visualization_msgs::Marker getPoseMarker(const geometry_msgs::Pose2D &state)
{
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = base_link_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.pose.position.x = state.x;
  marker_msg.pose.position.y = state.y;
  marker_msg.pose.position.z = 0;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 0.3;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.5;
  marker_msg.scale.x = 0.1;
  marker_msg.scale.y = 0.1;
  marker_msg.scale.z = 0.1;

  return marker_msg;
}

visualization_msgs::Marker getTextMarker(turtlebot_guide_msgs::FollowerState state)
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = base_link_;
  text_marker.header.stamp = ros::Time::now();
  text_marker.id = 0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = 2.0;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 0.0;
  text_marker.color.a = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.scale.x = 0.5;
  text_marker.scale.y = 0.5;
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

std::vector<geometry_msgs::Point32> getNonFollowingPoints()
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

std::vector<geometry_msgs::Point32> getFollowingPoints()
{
  std::vector<geometry_msgs::Point32> points;
  geometry_msgs::Point32 origin;
  origin.x = -0.8414; origin.y = 0.2944; origin.z = 0.0;
  points.push_back(origin);

  geometry_msgs::Point32 via1, via2, via3;
  via1.x = -2.8; via1.y = 1.21; via1.z = 0.0;
  points.push_back(via1);

  via2.x = -2.8; via2.y = -1.3; via2.z = 0.0;
  points.push_back(via2);

  via3.x = -0.8414; via3.y = -0.3944; via3.z = 0.0;
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
