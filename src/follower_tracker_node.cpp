#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "follower_tracker.h"

ros::Publisher pub;
rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& depth_msgs)
{
//  ros::Time start_time, end_time;
//  ros::Duration dura;
//  start_time = ros::Time::now();

  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  // convert the sensor message to point cloud type
  pcl_conversions::toPCL(*depth_msgs, *cloud);

  // down-sampling the point cloud
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> down_sampler;
  down_sampler.setInputCloud (cloudPtr);
  down_sampler.setLeafSize (0.01, 0.01, 0.01);

  pcl::PCLPointCloud2 *filtered_cloud = new pcl::PCLPointCloud2;
  down_sampler.filter (*filtered_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(*filtered_cloud, *cloud_p);

  // region of interest
  pcl::PassThrough<pcl::PointXYZRGB> roi_filter;
  roi_filter.setInputCloud(cloud_p);
  roi_filter.setFilterFieldName("z");
  roi_filter.setFilterLimits(0.4, 2.5);
  roi_filter.filter(*cloud_p);

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
  int nr_points = (int) cloud_p->points.size();
  while (cloud_p->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg_filter.setInputCloud (cloud_p);
    seg_filter.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_p);
    extract.setIndices (inliers);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*extract_object_cloud);
    cloud_p.swap (extract_object_cloud);
  }

  pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
  Eigen::Vector3f mass_center;
  feature_extractor.setInputCloud (extract_object_cloud);
  feature_extractor.compute ();
  feature_extractor.getMassCenter(mass_center);

  // x:height y:L/R z:distance
//  ROS_INFO_STREAM(mass_center);

//  end_time = ros::Time::now();
//  dura = end_time - start_time;
//  ROS_INFO_STREAM(dura);

  // convert the point cloud type to sensor message for publishing
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_p, output);

  pub.publish (output);
}

int main (int argc, char* argv[])
{
  // Initialize ROS
  ros::init (argc, argv, "follower_tracker");
  ros::NodeHandle nh;

  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/guidance/odom",
                                                                  "/turtlebot_follower_markers"));

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/guidance/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/guidance/output", 1);
  ROS_INFO("start publishing voxelized point cloud");

  // Spin
//  ros::Rate r(50);
//  while(ros::ok())
//  {
//    ros::spinOnce();
//    r.sleep();
//  }
  ros::spin();
}
