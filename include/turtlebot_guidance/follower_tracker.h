#ifndef FOLLOWER_TRACKER_H
#define FOLLOWER_TRACKER_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/Pose.h>
#include <gazebo_ir_camera_plugin/IRCamera.h>

namespace turtlebot_guidance
{

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

class FollowerTracker
{
public:

  FollowerTracker(const float &voxel_size);

  ~FollowerTracker();

  void filter(PointCloudPtr output);

  void setInputCloud(const PointCloud &input_cloud);

  void setROI(const double &azi);

private:

  const geometry_msgs::Pose getTargetPose();

  void extractTarget();

  ros::Publisher vis_pub_;

  PointCloudPtr cloud_p_;

  pcl::VoxelGrid<pcl::PointXYZRGB> down_sampler_filter_;
  pcl::PassThrough<pcl::PointXYZRGB> roi_filter_;

  const float voxel_size_;
};

} //namespace
#endif
