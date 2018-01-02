#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "turtlebot_follower_tracker/follower_tracker.h"

class FollowerTrackerNode
{

enum State { FOLLOWING, NOT_FOLLOWING, LOST };

public:

  FollowerTrackerNode():
    current_state_(FOLLOWING)
  {
    // Initialize the tracking class
    ft_.reset(new turtlebot_guide::FollowerTracker(0.01, "turtlebot.pcd"));

    // Create a ROS subscriber for the input point cloud and azimuthal angle
    //sub_ = nh_.subscribe ("/guidance/camera/depth/points", 100, &FollowerTrackerNode::pcCB, this);

    // Create a ROS publisher for the output point cloud and geometry pose
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/guidance/output", 10);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    ft_->getPointCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::copyPointCloud(*cloud, *color_cloud);

    ros::Duration(2.0).sleep();
    while(ros::ok()){
      sensor_msgs::PointCloud2 output_cloud_msg;
      pcl::toROSMsg(*cloud, output_cloud_msg);
      output_cloud_msg.header.frame_id = "base_link";
      output_cloud_msg.header.stamp = ros::Time::now();

      pc_pub_.publish (output_cloud_msg);
      ros::Duration(10.0).sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pc_pub_;

  State current_state_;

  std::unique_ptr<turtlebot_guide::FollowerTracker> ft_;
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "follower_tracker_test");
  FollowerTrackerNode node;

  ros::spin();
  return 0;
}
