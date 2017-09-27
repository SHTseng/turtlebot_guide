#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include "turtlebot_guidance/follower_tracker.h"

class FollowerTrackerNode
{
public:

  FollowerTrackerNode()
  {
    // Initialize the tracking class
    ft_.reset(new turtlebot_guidance::FollowerTracker(0.01));

    // Create a ROS subscriber for the input point cloud and azimuthal angle
    sub_ = nh_.subscribe ("/guidance/camera/depth/points", 100,
                                         &FollowerTrackerNode::trackCallback, this);

    ir_sub_ = nh_.subscribe("/guidance/ir_camera/scan", 100,
                                           &FollowerTrackerNode::IRCameraCallback, this);

    // Create a ROS publisher for the output point cloud and geometry pose
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/guidance/output", 10);

    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/guidance/follower_pose", 10);
  }

  void trackCallback(const sensor_msgs::PointCloud2ConstPtr &depth_msgs)
  {
    // convert the sensor message to point cloud type
    // sensor_msgs -> PointCloud2 -> PointCloud<PointXYZRGB>
    pcl::PCLPointCloud2 *pcl_pc2 = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*depth_msgs, *pcl_pc2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud_p);

    ft_->setInputCloud(*cloud_p);
    ft_->setROI(ir_sensor_msg_);

    // convert the point cloud type to sensor message for publishing
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ft_->filter(output_cloud);

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*output_cloud, output_cloud_msg);

    pc_pub_.publish (output_cloud_msg);
  }

  void IRCameraCallback(const gazebo_ir_camera_plugin::IRCamera &msg)
  {
    ir_sensor_msg_ = msg;
  }

private:

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  ros::Subscriber ir_sub_;

  ros::Publisher pc_pub_;

  ros::Publisher pose_pub_;

  std::unique_ptr<turtlebot_guidance::FollowerTracker> ft_;

  gazebo_ir_camera_plugin::IRCamera ir_sensor_msg_;

};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "follower_tracker_node");
  FollowerTrackerNode node;

  ros::spin();
}
