#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace turtlebot_guide
{
class MapOdomPublisher
{
public:
  MapOdomPublisher()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("init_x", current_x, 0.0);
    private_nh.param("init_y", current_y, 0.0);
    private_nh.param("init_th", current_th, 0.0);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 1000,
                                &MapOdomPublisher::cmdVelCB, this);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time  = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    ros::Rate r(1.0);
    while(nh.ok())
    {
      ros::spinOnce();
      current_time  = ros::Time::now();
      double dt = (current_time - last_time).toSec();
      double dx = (vx*cos(current_th)-vy*sin(current_th))*dt;
      double dy = (vx*sin(current_th)+vy*cos(current_th))*dt;
      double dth = vth*dt;

      current_x += dx;
      current_y += dy;
      current_th += dth;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_th);
      geometry_msgs::TransformStamped odom_transform;
      odom_transform.header.stamp = current_time;
      odom_transform.header.frame_id = "odom";
      odom_transform.child_frame_id = "base_footprint";

      odom_transform.transform.translation.x = current_x;
      odom_transform.transform.translation.y = current_y;
      odom_transform.transform.translation.z = 0.0;
      odom_transform.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_transform);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = current_x;
      odom.pose.pose.position.y = current_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub_.publish(odom);

      //Reset the vx, vy and vth to reduce error from receving /cmd_vel
      vx = 0.0;
      vy = 0.0;
      vth = 0.0;

      last_time = current_time;
      r.sleep();
    }
  }

  ~MapOdomPublisher()
  {
    cmd_vel_sub_.shutdown();
    odom_pub_.shutdown();
  }

private:

  void cmdVelCB(const geometry_msgs::Twist& msg)
  {
    vx = msg.linear.x;
    vy = msg.linear.y;
    vth = msg.angular.z;
  }

  ros::Publisher odom_pub_;
  ros::Subscriber cmd_vel_sub_;

  geometry_msgs::Twist vel_cmd_;

  double current_x, current_y, current_th;

  double vx;
  double vy;
  double vth;
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_odom_publisher");
  turtlebot_guide::MapOdomPublisher mopub;
//  ros::spin();
}
