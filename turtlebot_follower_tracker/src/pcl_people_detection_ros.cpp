#include <turtlebot_follower_tracker/pcl_people_detection_ros.h>

namespace turtlebot_guide
{

PCLPeopleDetectionROS::PCLPeopleDetectionROS(ros::NodeHandle& n, ros::NodeHandle& pnh):
  nh_(n), pnh_(pnh), base_frame_("base_footprint")
{
  // make sure the transfromation between camera frame and base frame exist
//  tf_listener_.lookupTransform("base_footprint", "camera_depth_optical_frame",
//                               ros::Time::now(), camera_base_tf_);
  double camera_height = 0.887;
  pnh_.param("camera_height", camera_height, 0.087);
  people_detector_.reset(new PCLPeopleDetection(camera_height));

  person_pub_ = nh_.advertise<spencer_tracking_msgs::DetectedPersons> ("output", 1,
                                                   boost::bind(&PCLPeopleDetectionROS::connectCB, this, _1),
                                                   boost::bind(&PCLPeopleDetectionROS::disconnectCB, this, _1));
}

PCLPeopleDetectionROS::~PCLPeopleDetectionROS()
{
  person_pub_.shutdown();
  sub_.shutdown();
}

void PCLPeopleDetectionROS::depthCB(const sensor_msgs::PointCloud2ConstPtr& depth_msgs)
{
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*depth_msgs, *pcl_input);

  PointCloud::Ptr cloud_p(new PointCloud);
  pcl::fromPCLPointCloud2(*pcl_input, *cloud_p);

  people_detector_->setInputCloud(*cloud_p);
  if(people_detector_->compute()){
    Eigen::Affine3f people_pose = people_detector_->getPeoplePose();
    Eigen::Quaternionf q(people_pose.rotation());

    geometry_msgs::PoseWithCovariance pose_msg;
    pose_msg.pose.position.x = -people_pose.translation().z();
    pose_msg.pose.position.y = people_pose.translation().x();
    pose_msg.pose.position.z = people_pose.translation().y();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_msg.covariance.fill(0.0);
    pose_msg.covariance[0 * 6 + 0] = AVERAGE_POSITION_VARIANCE; // x
    pose_msg.covariance[1 * 6 + 1] = AVERAGE_POSITION_VARIANCE; // y
    pose_msg.covariance[2 * 6 + 2] = AVERAGE_POSITION_VARIANCE; // z
    pose_msg.covariance[3 * 6 + 3] = INFINITE_VARIANCE; // x rotation
    pose_msg.covariance[4 * 6 + 4] = INFINITE_VARIANCE; // y rotation
    pose_msg.covariance[5 * 6 + 5] = INFINITE_VARIANCE; // z rotation

    spencer_tracking_msgs::DetectedPerson person_msg;
    person_msg.detection_id = 0;
    person_msg.confidence = 1.0;
    person_msg.pose = pose_msg;
    person_msg.modality = "rgbd";

    spencer_tracking_msgs::DetectedPersons persons_msg;
    persons_msg.header.frame_id = base_frame_;
    persons_msg.header.stamp = ros::Time::now();
    persons_msg.detections.push_back(person_msg);
    person_pub_.publish(persons_msg);
  }

  ros::Time current_t = ros::Time::now();
  ROS_INFO_STREAM("Loop time: " << (current_t-last_time_).toSec() << "sec...");
//  if ((current_t-last_time_).toSec() < 5.0){
//    ROS_INFO_STREAM("sleep " << (current_t-last_time_).toSec() << " secs...");
//    ros::Duration((current_t-last_time_).toSec()).sleep();
//  }

  last_time_ = ros::Time::now();
}

void PCLPeopleDetectionROS::connectCB(const ros::SingleSubscriberPublisher &pub)
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  try{
      tf_listener_.waitForTransform("base_link", "camera_depth_frame", ros::Time(0), ros::Duration(10.0));
  }
  catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
  }

  if (!sub_ && person_pub_.getNumSubscribers() > 0)
  {
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("input", 1, &PCLPeopleDetectionROS::depthCB, this);
    last_time_ = ros::Time::now();
    ROS_INFO("Start follower tracker callback");
  }
}

void PCLPeopleDetectionROS::disconnectCB(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (person_pub_.getNumSubscribers() == 0)
  {
    sub_.shutdown();
  }
}

}
