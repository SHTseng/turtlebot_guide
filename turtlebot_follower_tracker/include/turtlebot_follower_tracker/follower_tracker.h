#ifndef FOLLOWER_TRACKER_H
#define FOLLOWER_TRACKER_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/Pose.h>
#include <gazebo_ir_camera_plugin/IRCamera.h>

#include <turtlebot_follower_tracker/mesh_model_loader.h>

namespace turtlebot_guide
{

//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

class FollowerTracker
{
public:

  FollowerTracker(const float &voxel_size, const std::string& mesh_model_name);

  ~FollowerTracker();

  void filter(PointCloudPtr output);

  void setInputCloud(const PointCloud &input_cloud);

  void setROI(const double &azi);

  void getPointCloud(PointCloudPtr &output_cloud);

  const geometry_msgs::Pose getTargetPose() { return tracked_pose_; }

private:

  void alignTarget();

  void extractTarget();

  void loadTargetModel();

  geometry_msgs::Pose tracked_pose_;
  geometry_msgs::Pose previous_pose_;

  PointCloudPtr scene_cloud_;
  pcl::VoxelGrid<PointT> down_sampler_filter_;
  pcl::PassThrough<PointT> roi_filter_;
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  const float voxel_size_;

  std::shared_ptr<MeshModelLoader> mesh_loader_;
  std::string mesh_model_name_;
  PointCloudPtr model_cloud_;

};

} //namespace
#endif
