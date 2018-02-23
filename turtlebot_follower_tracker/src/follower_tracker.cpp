#include "turtlebot_follower_tracker/follower_tracker.h"

namespace turtlebot_guide
{

FollowerTracker::FollowerTracker(const float &voxel_size, const std::string& mesh_model_name, const float &initial_y):
  voxel_size_(voxel_size),
  initial_y_(initial_y),
  mesh_model_name_(mesh_model_name)
{
  mesh_loader_.reset(new MeshModelLoader(mesh_model_name_));
  this->loadTargetModel();

  Eigen::Affine3f model_transform = Eigen::Affine3f::Identity();

  // Initial guess for follower at back
//  model_transform.translation() << 0.0, 0.2, 0.0;
//  model_transform.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));
//  model_transform.rotate (Eigen::AngleAxisf (-M_PI/2, Eigen::Vector3f::UnitZ()));

  // Initial guess for CCN project
  model_transform.translation() << 0.3, 0.29, 1.8;
  model_transform.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));
  model_transform.rotate (Eigen::AngleAxisf (-M_PI, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*model_cloud_, *model_cloud_, model_transform);

  previous_pose_.position.x = 0.3;
  previous_pose_.position.z = 1.8;
}

FollowerTracker::~FollowerTracker()
{

}

void FollowerTracker::filter(PointCloudPtr output_cloud)
{
  this->extractTarget();

  pcl::StatisticalOutlierRemoval<PointT> outlier_removal(true);
  outlier_removal.setInputCloud(scene_cloud_);
  outlier_removal.setMeanK(2);
  outlier_removal.setStddevMulThresh(0.0);
  outlier_removal.setNegative(0);
  outlier_removal.setKeepOrganized(false);
  outlier_removal.filter(*scene_cloud_);

  this->alignTarget();
  //update initial guess
//  updateICPInitialGuess(tracked_pose_.position.x-previous_pose_.position.x, tracked_pose_.position.z-previous_pose_.position.z);

  previous_pose_ = tracked_pose_;
  pcl::copyPointCloud(*scene_cloud_, *output_cloud);
}

void FollowerTracker::setROI(const double &azi)
{
  float offset = 0.4;
//  float mapped_pt = 3.031*std::tan(azi);
  float mapped_pt = azi;

  roi_filter_.setInputCloud(scene_cloud_);
  roi_filter_.setFilterFieldName("x");
//  roi_filter_.setFilterLimits(-mapped_pt-offset, -mapped_pt+offset);
  roi_filter_.setFilterLimits(mapped_pt-offset, mapped_pt+offset);
  roi_filter_.filter(*scene_cloud_);

  roi_filter_.setInputCloud(scene_cloud_);
  roi_filter_.setFilterFieldName("z");
  roi_filter_.setFilterLimits(0.4, 2.0);
  roi_filter_.filter(*scene_cloud_);
}

void FollowerTracker::setInputCloud(const PointCloud &input_cloud)
{
  scene_cloud_.reset(new PointCloud(input_cloud));

  // down-sampling the point cloud
  down_sampler_filter_.setInputCloud (scene_cloud_);
  down_sampler_filter_.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
  down_sampler_filter_.filter (*scene_cloud_);
}

void FollowerTracker::alignTarget()
{
  icp.setInputSource(model_cloud_);
  icp.setInputTarget(scene_cloud_);
  icp.setMaximumIterations(15);
  PointCloud final;
  icp.align(final);
  if(icp.hasConverged()){
    Eigen::Matrix4d tracked_p = icp.getFinalTransformation().cast<double>();
    Eigen::Affine3d tracked_p_aff;
    tracked_p_aff.matrix() = tracked_p;
    tf::poseEigenToMsg(tracked_p_aff, tracked_pose_);
  }
}

void FollowerTracker::updateICPInitialGuess(const float& x, const float &z)
{
  Eigen::Affine3f model_transform = Eigen::Affine3f::Identity();
  ROS_INFO_STREAM(x);
  model_transform.translation() << -x, 0, 0;
  model_transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud (*model_cloud_, *model_cloud_, model_transform);
}

void FollowerTracker::extractTarget()
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg_filter;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr extract_object_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg_filter.setOptimizeCoefficients (true);
  seg_filter.setModelType (pcl::SACMODEL_PLANE);
  seg_filter.setMethodType (pcl::SAC_RANSAC);
  seg_filter.setMaxIterations (1000);
  seg_filter.setDistanceThreshold (0.02);

  pcl::ExtractIndices<PointT> extract;
  int nr_points = (int) scene_cloud_->points.size();
  while (scene_cloud_->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg_filter.setInputCloud (scene_cloud_);
    seg_filter.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the inliers
    extract.setInputCloud (scene_cloud_);
    extract.setIndices (inliers);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*extract_object_cloud);
    scene_cloud_.swap (extract_object_cloud);
  }
}

void FollowerTracker::loadTargetModel()
{
  model_cloud_.reset(new PointCloud());
  PointCloud cloud;
  if(!mesh_loader_->readFile(cloud)){
    ROS_ERROR("Load tracking target mesh file fail");
    return;
  }
  pcl::copyPointCloud(cloud, *model_cloud_);
}

void FollowerTracker::getPointCloud(PointCloudPtr& output_cloud)
{
  this->loadTargetModel();
//  output_cloud = model_cloud_;
  pcl::copyPointCloud(*model_cloud_, *output_cloud);
}

} // namesapce
