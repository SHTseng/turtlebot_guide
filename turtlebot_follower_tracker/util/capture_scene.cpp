#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

sensor_msgs::PointCloud2 data;

void callback(const sensor_msgs::PointCloud2& msg)
{
  data = msg;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "capture_scene");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe("/turtlebot_follower_tracker/output", 1, callback);
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);

  while(sub.getNumPublishers() == 0){
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  ros::spinOnce();
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(data, *pcl_input);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromPCLPointCloud2(*pcl_input, *cloud_p);

  std::string save_file_name = ros::package::getPath("turtlebot_follower_tracker")+"/data/scene.pcd";
  if(pcl::io::savePCDFile(save_file_name, *cloud_p) == -1)
    ROS_ERROR("Save pcd file fail");

  return 0;
}
