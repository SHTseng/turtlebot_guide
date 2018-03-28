#ifndef FOLLOWER_TRACKER_ROS_H
#define FOLLOWER_TRACKER_ROS_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread/mutex.hpp>
#include <memory>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <turtlebot_follower_tracker/follower_tracker.h>
//#include <rviz_visual_tools/rviz_visual_tools.h>

namespace turtlebot_guide
{
class FollowerTrackerROS
{
public:

  FollowerTrackerROS(ros::NodeHandle& n, ros::NodeHandle& pnh);

  ~FollowerTrackerROS();

private:

  void depthCB(const sensor_msgs::PointCloud2ConstPtr& depth_msgs);

  void connectCB(const ros::SingleSubscriberPublisher& pub);

  void disconnectCB(const ros::SingleSubscriberPublisher& pub);

  void publishBoundingBox(const geometry_msgs::Pose& msg);

  void frameTransformation(const geometry_msgs::Pose& camera_frame,
                           geometry_msgs::Pose& transformed_frame);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_;
  ros::Publisher pose_pub_;
  ros::Publisher cloud_pub_;
  boost::mutex connect_mutex_;

  ros::Time last_time_;

  std::string mesh_name_;
  std::shared_ptr<turtlebot_guide::FollowerTracker> ft_;
  geometry_msgs::PoseStamped target_pose_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform camera_base_tf_;

  /// 0. camera look in back
  /// 1. camera look in front
  int camera_mode_;

//  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};
}

#endif // FOLLOWER_TRACKER_ROS_H
