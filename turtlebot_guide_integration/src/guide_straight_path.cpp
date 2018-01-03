#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "guide_straight_path");
  ros::NodeHandle nh;

  ros::Publisher guide_cmdvel_pub =
      nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 5);
  ros::Publisher follower_cmdvel_pub =
      nh.advertise<geometry_msgs::Twist>("follower/cmd_vel_mux/input/teleop", 5);

  geometry_msgs::Twist control_speed;
  control_speed.linear.x = 0.3;
  control_speed.linear.y = 0.0;
  control_speed.angular.z = 0.0;

  ros::Time start_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  while ((last_time-start_time).toSec() < 10.0){
    guide_cmdvel_pub.publish(control_speed);
    follower_cmdvel_pub.publish(control_speed);
    last_time = ros::Time::now();
  }

  control_speed.linear.x = 0.0;
  control_speed.linear.y = 0.0;
  control_speed.angular.z = 0.0;
  guide_cmdvel_pub.publish(control_speed);
  follower_cmdvel_pub.publish(control_speed);

  return 0;
}
