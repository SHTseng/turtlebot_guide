#include "turtlebot_guidance/follower_tracker.h"

namespace turtlebot_guidance
{

FollowerTracker::FollowerTracker(const float &voxel_size):
  voxel_size_(voxel_size)
{

}

FollowerTracker::~FollowerTracker()
{

}

void FollowerTracker::filter(PointCloudPtr output_cloud)
{
  this->extractTarget();
  pcl::copyPointCloud(*cloud_p_, *output_cloud);
}

void FollowerTracker::setROI(gazebo_ir_camera_plugin::IRCamera ir_msg)
{
  roi_filter_.setInputCloud(cloud_p_);
  roi_filter_.setFilterFieldName("x");
  roi_filter_.setFilterLimits(-0.3, 0.3);
  roi_filter_.filter(*cloud_p_);

  roi_filter_.setInputCloud(cloud_p_);
  roi_filter_.setFilterFieldName("z");
  roi_filter_.setFilterLimits(0.4, 2.5);
  roi_filter_.filter(*cloud_p_);
}

void FollowerTracker::setInputCloud(const PointCloud &input_cloud)
{
  cloud_p_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(input_cloud));

  // down-sampling the point cloud
  down_sampler_filter_.setInputCloud (cloud_p_);
  down_sampler_filter_.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
  down_sampler_filter_.filter (*cloud_p_);
}

void FollowerTracker::extractTarget()
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg_filter;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_object_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg_filter.setOptimizeCoefficients (true);
  seg_filter.setModelType (pcl::SACMODEL_PLANE);
  seg_filter.setMethodType (pcl::SAC_RANSAC);
  seg_filter.setMaxIterations (1000);
  seg_filter.setDistanceThreshold (0.02);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  int nr_points = (int) cloud_p_->points.size();
  while (cloud_p_->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg_filter.setInputCloud (cloud_p_);
    seg_filter.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_p_);
    extract.setIndices (inliers);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*extract_object_cloud);
    cloud_p_.swap (extract_object_cloud);
  }
}

} // namesapce
