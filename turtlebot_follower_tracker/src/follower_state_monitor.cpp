#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#include <boost/thread.hpp>
#include <memory>

namespace turtlebot_guide
{

enum FollowerState
{
  FOLLOWING,
  NOT_FOLLOWING,
  STOP
};

class FollowerStateMonitor
{

public:

  FollowerStateMonitor():
    wait_for_wake_(true)
  {
    ros::NodeHandle private_nh("~");

    std::vector<double> following_range;
    private_nh.param("following_range", following_range, std::vector<double>());

    for(int i = 0; i < following_range.size(); i+=2)
    {
      geometry_msgs::Point32 p;
      p.x = following_range[i];
      p.y = following_range[i+1];
      following_polygon_.push_back(p);
    }

    monitor_thread_ = new boost::thread(boost::bind(&FollowerStateMonitor::monitorThread, this));

    odom_sub_ = nh_.subscribe("odom", 10, &FollowerStateMonitor::odomCB, this);

    track_sub_ = nh_.subscribe("spencer/perception/tracked_persons", 10, &FollowerStateMonitor::trackCB, this);
  }

  ~FollowerStateMonitor()
  {
    monitor_thread_->interrupt();
    monitor_thread_->join();

    delete monitor_thread_;
  }

private:

  void odomCB(const nav_msgs::Odometry::ConstPtr _msg)
  {
    boost::recursive_mutex::scoped_lock lock(monitor_mutex_);
    odom_ = *_msg;
  }

  void trackCB(const spencer_tracking_msgs::TrackedPersons::ConstPtr _msg)
  {
    if (_msg->tracks.size() == 0)
    {
      return;
    }

    boost::recursive_mutex::scoped_lock lock(monitor_mutex_);
    follower_pose_ = _msg->tracks.front().pose.pose;
    follower_vel_ = _msg->tracks.front().twist.twist;

    wait_for_wake_ = false;
    monitor_cond_.notify_one();
  }

  void monitorThread()
  {
    ros::NodeHandle n;
//    boost::mutex::scoped_lock lock(monitor_mutex_);
    boost::unique_lock<boost::recursive_mutex> lock(monitor_mutex_);
    while(n.ok())
    {
      while(wait_for_wake_)
      {
        monitor_cond_.wait(lock);
      }

      double d = distance(odom_.pose.pose, follower_pose_);
      FollowerState state = checkFollowerState();

      wait_for_wake_ = true;
    }
  }

  FollowerState checkFollowerState()
  {
//    boost::recursive_mutex::scoped_lock lock(monitor_mutex_);
    geometry_msgs::Point32 follower_p;
    follower_p.x = follower_pose_.position.x;
    follower_p.y = follower_pose_.position.y;
    ROS_INFO_STREAM(follower_p.x << " " << follower_p.y);

    // Coordinate transform of the following range

    if(insidePolygon(following_polygon_, follower_p))
      return FOLLOWING;
    else
      return NOT_FOLLOWING;
  }

  bool insidePolygon(const std::vector<geometry_msgs::Point32> &polygon, const geometry_msgs::Point32 &p)
  {
    // Calculate each segment of polyong counter clockwise and check the point lies in the left side of the line
    bool left_side = true;
    int area;
    for (int i = 0; i < polygon.size()-1; i++)
    {
      // outer product to calculate area
      area = (polygon[i].x-p.x)*(polygon[i+1].y-p.y)-(polygon[i+1].x-p.x)*(polygon[i].y-p.y);
      // if the value with different sign indicates the point lies in the right side
      if(area < 0) left_side = false;
    }

    // Calculate the last point and first point
    area = (polygon.back().x-p.x)*(polygon.front().y-p.y)-(polygon.front().x-p.x)*(polygon.back().y-p.y);
    if(area < 0) left_side = false;

    return left_side;
  }

  double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
  {
    return hypot(p1.position.x-p2.position.x, p1.position.y-p2.position.y);
  }

  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber track_sub_;

  nav_msgs::Odometry odom_;

  geometry_msgs::Pose follower_pose_;
  geometry_msgs::Twist follower_vel_;

  FollowerState follower_state_;
  std::vector<geometry_msgs::Point32> following_polygon_;
  std::vector<geometry_msgs::Point32> camera_polygon_;

  boost::thread *monitor_thread_;
  boost::recursive_mutex monitor_mutex_;
  boost::condition_variable_any monitor_cond_;

  bool wait_for_wake_;

};


} // end namespace

typedef std::shared_ptr<turtlebot_guide::FollowerStateMonitor> FollowerStateMonitorPtr;

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "follower_state_monitor");
  FollowerStateMonitorPtr monitor(new turtlebot_guide::FollowerStateMonitor());

  ros::spin();
  return 0;
}
