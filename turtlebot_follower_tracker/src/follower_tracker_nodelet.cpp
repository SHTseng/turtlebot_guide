//#include <turtlebot_follower_tracker/follower_tracker_ros.h>
#include <turtlebot_follower_tracker/pcl_people_detection_ros.h>

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
    ROS_INFO("Load follower tracker nodelet");
//    ftr_.reset(new FollowerTrackerROS(getNodeHandle(), getPrivateNodeHandle()));
    pdr_.reset(new PCLPeopleDetectionROS(getNodeHandle(), getPrivateNodeHandle()));
  }

//  std::shared_ptr<turtlebot_guide::FollowerTrackerROS> ftr_;
  std::shared_ptr<turtlebot_guide::PCLPeopleDetectionROS> pdr_;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(turtlebot_guide, FollowerTrackerNodelet, turtlebot_guide::FollowerTrackerNodelet, nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(turtlebot_guide::FollowerTrackerNodelet, nodelet::Nodelet);
