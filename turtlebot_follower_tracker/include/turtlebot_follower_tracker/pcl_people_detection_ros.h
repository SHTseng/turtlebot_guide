#ifndef PCL_PEOPLE_DETECTION_ROS_H
#define PCL_PEOPLE_DETECTION_ROS_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <spencer_tracking_msgs/DetectedPersons.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <turtlebot_follower_tracker/pcl_people_detectioin.h>

#include <boost/thread/mutex.hpp>
#include <memory>

namespace turtlebot_guide
{

const double AVERAGE_POSITION_VARIANCE = 0.05; //pow(0.17 /* stddev in meters */, 2)
const double INFINITE_VARIANCE = std::numeric_limits<double>::infinity();

class PCLPeopleDetectionROS
{
public:

  PCLPeopleDetectionROS(ros::NodeHandle& n, ros::NodeHandle& pnh);

  ~PCLPeopleDetectionROS();

private:

  void depthCB(const sensor_msgs::PointCloud2ConstPtr& depth_msgs);

  void connectCB(const ros::SingleSubscriberPublisher& pub);

  void disconnectCB(const ros::SingleSubscriberPublisher& pub);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_;
  ros::Publisher pose_pub_;
  ros::Publisher person_pub_;
  boost::mutex connect_mutex_;

  ros::Time last_time_;

  std::string base_frame_;

  std::shared_ptr<PCLPeopleDetection> people_detector_;

  geometry_msgs::PoseStamped target_pose_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform camera_base_tf_;

};
}

#endif // PCL_PEOPLE_DETECTION_ROS_H
