#include <turtlebot_follower_tracker/follower_tracker_ros.h>

using namespace turtlebot_guide;

FollowerTrackerROS::FollowerTrackerROS(ros::NodeHandle &n, ros::NodeHandle &pnh):
  nh_(n), pnh_(pnh)
{
  mesh_name_ = "turtlebot.pcd";
//  pnh.param<std::string>("mesh_name", mesh_name);

  ft_.reset(new FollowerTracker(0.015, mesh_name_));
  ros::Duration(1.0).sleep();

  // make sure the transfromation between camera frame and base frame exist
  tf_listener_.lookupTransform("base_footprint", "camera_depth_optical_frame",
                               ros::Time::now(), camera_base_tf_);

  // prepare rviz visual tools for visualizing the bounding box
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers"));
  visual_tools_->loadMarkerPub();

  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // initial the tracked pose
  target_pose_.pose.position.x = 0.0;
  target_pose_.pose.position.y = 0.0;
  target_pose_.pose.position.z = 0.0;

  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("follower_tracker/tracked_pose", 1);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output", 1,
                                                   boost::bind(&FollowerTrackerROS::connectCB, this, _1),
                                                   boost::bind(&FollowerTrackerROS::disconnectCB, this, _1));
}

FollowerTrackerROS::~FollowerTrackerROS()
{
  sub_.shutdown();
}

void FollowerTrackerROS::depthCB(const sensor_msgs::PointCloud2ConstPtr& depth_msgs)
{
  // convert the sensor message to point cloud type
  // sensor_msgs -> PointCloud2 -> PointCloud<PointXYZRGB>
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*depth_msgs, *pcl_input);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*pcl_input, *cloud_p);

//  double azi = fusedSensorData();
  ft_->setInputCloud(*cloud_p);
  ft_->setROI(target_pose_.pose.position.y);
//  ft_->setROI(0);

  // convert the point cloud type to sensor message for publishing
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ft_->filter(output_cloud);

  sensor_msgs::PointCloud2 output_cloud_msg;
  pcl::toROSMsg(*output_cloud, output_cloud_msg);
  cloud_pub_.publish(output_cloud_msg);

  geometry_msgs::Pose camera_frame_pose = ft_->getTargetPose();
  target_pose_.header.frame_id = "camera_depth_optical_frame";
  target_pose_.header.stamp = ros::Time::now();

  geometry_msgs::Pose transformed_pose;
  transformed_pose.position.x = -camera_frame_pose.position.z-0.117;
  transformed_pose.position.y = camera_frame_pose.position.x-0.052;
  transformed_pose.position.z = 0.21;
  publishBoundingBox(transformed_pose);

  target_pose_.pose = transformed_pose;
  pose_pub_.publish(camera_frame_pose);

//  ROS_INFO_STREAM(target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z);
  ros::Duration current_time = ros::Time::now() - last_time_;
  ROS_INFO_STREAM(current_time.toSec()*1000);
  last_time_ = ros::Time::now();
}

void FollowerTrackerROS::connectCB(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  try{
      tf_listener_.waitForTransform("base_link", "camera_depth_frame", ros::Time(0), ros::Duration(10.0));
  }
  catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
  }

  if (!sub_ && cloud_pub_.getNumSubscribers() > 0)
  {
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("input", 1, &FollowerTrackerROS::depthCB, this);
    last_time_ = ros::Time::now();
    ROS_INFO("start follower tracker callback");
  }
}

void FollowerTrackerROS::disconnectCB(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (cloud_pub_.getNumSubscribers() == 0)
  {
    sub_.shutdown();
  }
}

void FollowerTrackerROS::publishBoundingBox(const geometry_msgs::Pose& msg)
{
  visual_tools_->deleteAllMarkers();

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << msg.position.x, msg.position.y, msg.position.z;
  visual_tools_->publishWireframeCuboid(pose, 0.46, 0.354, 0.354, rviz_visual_tools::YELLOW);
  visual_tools_->trigger();
}
