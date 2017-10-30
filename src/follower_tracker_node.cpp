#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include "turtlebot_guidance/follower_tracker.h"

class FollowerTrackerNode
{

enum State { FOLLOWING, NOT_FOLLOWING, LOST };

public:

  FollowerTrackerNode():
    current_state_(FOLLOWING)
  {
    // Initialize the tracking class
    ft_.reset(new turtlebot_guidance::FollowerTracker(0.01));

    // Create a ROS subscriber for the input point cloud and azimuthal angle
    sub_ = nh_.subscribe ("/guidance/camera/depth/points", 100,
                                         &FollowerTrackerNode::trackCallback, this);

    ir1_sub_ = nh_.subscribe("/guidance/ir_camera_mid/scan", 10,
                                           &FollowerTrackerNode::IRCameraCallback, this);

    ir2_sub_ = nh_.subscribe("/guidance/ir_camera_l/scan", 10,
                                           &FollowerTrackerNode::IRCamera2Callback, this);

    ir3_sub_ = nh_.subscribe("/guidance/ir_camera_3/scan", 10,
                                           &FollowerTrackerNode::IRCamera3Callback, this);

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

    double azi = fusedSensorData();
    ft_->setInputCloud(*cloud_p);
    ft_->setROI(azi);

    // convert the point cloud type to sensor message for publishing
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ft_->filter(output_cloud);

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*output_cloud, output_cloud_msg);

    pc_pub_.publish (output_cloud_msg);
  }

  void IRCameraCallback(const gazebo_ir_camera_plugin::IRCamera &msg)
  {
    ir_camera_1_msg_ = msg;
  }

  void IRCamera2Callback(const gazebo_ir_camera_plugin::IRCamera &msg)
  {
    ir_camera_2_msg_ = msg;
  }

  void IRCamera3Callback(const gazebo_ir_camera_plugin::IRCamera &msg)
  {
    ir_camera_3_msg_ = msg;
  }

private:

  double fusedSensorData()
  {
    switch(current_state_)
    {
      case 0:
        ROS_INFO_STREAM("Following");
        break;
      case 1:
        ROS_INFO_STREAM("Not_Following");
        break;
      case 2:
        ROS_INFO_STREAM("Lost");
        break;
    }
    double azi = 0.0;
    if (ir_camera_1_msg_.azimuthal_angles.size() > 0)
    {
      azi = ir_camera_1_msg_.azimuthal_angles.front();
      current_state_ = FOLLOWING;
      ir_camera_1_msg_.azimuthal_angles.clear();
      return azi;
    }

    if (ir_camera_2_msg_.azimuthal_angles.size() > 0 || ir_camera_3_msg_.azimuthal_angles.size() > 0)
    {
      current_state_ = NOT_FOLLOWING;
      ir_camera_2_msg_.azimuthal_angles.clear();
      ir_camera_3_msg_.azimuthal_angles.clear();
      return azi;
    }

    if (ir_camera_1_msg_.azimuthal_angles.size() == 0 &&
        ir_camera_2_msg_.azimuthal_angles.size() == 0 &&
        ir_camera_3_msg_.azimuthal_angles.size() == 0)
    {
      current_state_ = LOST;
      return azi;
    }
  }

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  ros::Subscriber ir1_sub_;
  ros::Subscriber ir2_sub_;
  ros::Subscriber ir3_sub_;

  ros::Publisher pc_pub_;
  ros::Publisher pose_pub_;

  State current_state_;

  std::unique_ptr<turtlebot_guidance::FollowerTracker> ft_;

  gazebo_ir_camera_plugin::IRCamera ir_camera_1_msg_;
  gazebo_ir_camera_plugin::IRCamera ir_camera_2_msg_;
  gazebo_ir_camera_plugin::IRCamera ir_camera_3_msg_;

};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "follower_tracker_node");
  FollowerTrackerNode node;

//  ros::spin();

  while (ros::ok())
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
}
