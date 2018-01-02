#include <turtlebot_follower_tracker/follower_tracker_ros.h>

namespace turtlebot_guide
{

class FollowerTrackerNodelet : public nodelet::Nodelet
{
public:

  FollowerTrackerNodelet(){}

  ~FollowerTrackerNodelet(){}

private:

  virtual void onInit()
  {
    ftr_.reset(new FollowerTrackerROS(getNodeHandle(), getPrivateNodeHandle()));
  }

  std::shared_ptr<turtlebot_guide::FollowerTrackerROS> ftr_;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(turtlebot_guide, FollowerTrackerNodelet, turtlebot_guide::FollowerTrackerNodelet, nodelet::Nodelet);
