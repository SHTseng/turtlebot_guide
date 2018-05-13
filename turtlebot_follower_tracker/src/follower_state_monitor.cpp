#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <turtlebot_guide_msgs/FollowerState.h>

#include <boost/thread.hpp>
#include <memory>

namespace turtlebot_guide
{

enum FollowerState
{
  FOLLOWING,
  NOT_FOLLOWING,
  STOPPED,
  UNKNOWN
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

    for(std::size_t i = 0; i < following_range.size(); i+=2)
    {
      geometry_msgs::Point p;
      p.x = following_range[i];
      p.y = following_range[i+1];
      following_polygon_.push_back(p);
    }

    monitor_thread_ = new boost::thread(boost::bind(&FollowerStateMonitor::monitorThread, this));

    odom_sub_ = nh_.subscribe("odom", 10, &FollowerStateMonitor::odomCB, this);

    track_sub_ = nh_.subscribe("spencer/perception/tracked_persons", 10, &FollowerStateMonitor::trackCB, this);

    state_pub_ = nh_.advertise<turtlebot_guide_msgs::FollowerState>("follower_state", 1000);
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
    boost::mutex::scoped_lock lock(monitor_mutex_);
    odom_ = *_msg;
  }

  void trackCB(const spencer_tracking_msgs::TrackedPersons::ConstPtr _msg)
  {
    boost::mutex::scoped_lock lock(monitor_mutex_);
    tracked_people_ = *_msg;

    wait_for_wake_ = false;
    monitor_cond_.notify_one();
  }

  void monitorThread()
  {
    ros::NodeHandle n;
    while(n.ok())
    {
      boost::unique_lock<boost::mutex> lock(monitor_mutex_);
      while(wait_for_wake_)
      {
        monitor_cond_.wait(lock);
      }

      FollowerState follower_state;
      if (tracked_people_.tracks.empty())
      {
        follower_state = UNKNOWN;
      }
      else
      {
        // Currently get the only track out
        follower_pose_ = tracked_people_.tracks.front().pose.pose;
        follower_vel_ = tracked_people_.tracks.front().twist.twist;

        // Check the follower is following or not
        follower_state = checkFollowerState();
      }

      // fill in the msg data
      turtlebot_guide_msgs::FollowerState state_msg;
      state_msg.header.frame_id = "base_footprint";
      state_msg.header.stamp = ros::Time::now();
      state_msg.state = follower_state;
      state_msg.pose.x = follower_pose_.position.x;
      state_msg.pose.y = follower_pose_.position.y;

      tf::Quaternion q(follower_pose_.orientation.x, follower_pose_.orientation.y,
                       follower_pose_.orientation.z, follower_pose_.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      state_msg.pose.theta = yaw;

      state_msg.vel.x = follower_vel_.linear.x;
      state_msg.vel.y = follower_vel_.linear.y;
      state_msg.vel.theta = follower_vel_.angular.z;

      state_pub_.publish(state_msg);

      // Record the follower pose for next run and unlock the mutex
      prev_follower_pose_ = follower_pose_;
      wait_for_wake_ = true;
    } // end while
  }

  FollowerState checkFollowerState()
  {
    // Check the follower is moving
    if (!isMoving())
      return STOPPED;

    return isFollowing() ? FOLLOWING : NOT_FOLLOWING;
  }

  bool isFollowing()
  {
    // coordinate transformation of the following range
    geometry_msgs::Point transformed_pose = transformToLocalFrame(follower_pose_.position);

    // make sure the follower is inside the polygon and the direction of vel is point to the robot
    if (insidePolygon(following_polygon_, transformed_pose))
      return true;
    else
      return false;
  }

  bool insidePolygon(const std::vector<geometry_msgs::Point> &polygon, const geometry_msgs::Point &p)
  {
    // Calculate each segment of polyong counter clockwise and check the point lies in the left side of the line
    bool left_side = true;
    double area;
    for (std::size_t i = 0; i < polygon.size()-1; i++)
    {
      // outer product to calculate area surrounded by point and two adjent vertices
      area = (polygon[i].x-p.x)*(polygon[i+1].y-p.y)-(polygon[i+1].x-p.x)*(polygon[i].y-p.y);
      // if the value with different sign indicates the point lies in the right side
      if(area < 0.0) left_side = false;
    }

    // Calculate the last point and first point
    area = (polygon.back().x-p.x)*(polygon.front().y-p.y)-(polygon.front().x-p.x)*(polygon.back().y-p.y);
    if(area < 0.0) left_side = false;

    return left_side;
  }

  geometry_msgs::Point transformToLocalFrame(const geometry_msgs::Point &p)
  {
    geometry_msgs::Point transformed_p;
    // hacks to transform the frame to odom, TODO: use tf
    transformed_p.x = p.x - odom_.pose.pose.position.x;
    transformed_p.y = p.y - odom_.pose.pose.position.y;
    return transformed_p;
  }

  bool isMoving()
  {
    double d = distance(follower_pose_, prev_follower_pose_);
    double vel_norm = hypot(follower_vel_.linear.x, follower_vel_.linear.y);
    return (d < 0.01 && vel_norm < 0.01) ? false : true;
  }

  double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
  {
    return hypot(p1.position.x-p2.position.x, p1.position.y-p2.position.y);
  }

  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber track_sub_;

  ros::Publisher state_pub_;

  nav_msgs::Odometry odom_;
  spencer_tracking_msgs::TrackedPersons tracked_people_;

  geometry_msgs::Pose follower_pose_;
  geometry_msgs::Pose prev_follower_pose_;
  geometry_msgs::Twist follower_vel_;

  std::vector<geometry_msgs::Point> following_polygon_;
  std::vector<geometry_msgs::Point> camera_polygon_;

  boost::thread *monitor_thread_;
  boost::mutex monitor_mutex_;
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
