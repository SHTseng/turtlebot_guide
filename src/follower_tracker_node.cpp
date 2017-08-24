#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>

#include "follower_tracker.h"

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& depth_msgs)
{
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  // convert the sensor message to point cloud type
  pcl_conversions::toPCL(*depth_msgs, *cloud);

  // down-sampling the point cloud
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> down_sampler;
  down_sampler.setInputCloud (cloudPtr);
  down_sampler.setLeafSize (0.02, 0.02, 0.02);

  pcl::PCLPointCloud2 filtered_cloud;
  down_sampler.filter (filtered_cloud);

  // convert the point cloud type to sensor message for publishing
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(filtered_cloud, output);

  pub.publish (output);
}

int main (int argc, char* argv[])
{
  // Initialize ROS
  ros::init (argc, argv, "follower_tracker");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/guidance/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/guidance/output", 1);
  ROS_INFO("start publishing voxelized point cloud");

  // Spin
  ros::spin();
}
