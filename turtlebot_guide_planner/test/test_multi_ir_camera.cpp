#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>

#include <gazebo_ir_camera_plugin/IRCamera.h>

#include <gtest/gtest.h>

int cnt = 0, ir2_cnt = 0, ir3_cnt = 0;

void callBack(const gazebo_ir_camera_plugin::IRCameraConstPtr &msg)
{
  cnt = msg->azimuthal_angles.size();
}

void callBack2(const gazebo_ir_camera_plugin::IRCameraConstPtr &msg)
{
  ir2_cnt = msg->azimuthal_angles.size();
}

void callBack3(const gazebo_ir_camera_plugin::IRCameraConstPtr &msg)
{
  ir3_cnt = msg->azimuthal_angles.size();
}

bool inTolerance(const double &val,
                 const double &ref,
                 const double &range)
{
  return val < (ref+range) && val > (ref-range) ? true : false;
}

TEST(StateDetectionTest, spawnRobot)
{
  ros::NodeHandle nh("");
  ros::service::waitForService("/gazebo/pause_physics", 1.0);
  ros::ServiceClient pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty empty_srv;
  ASSERT_TRUE(pause_physics_client.call(empty_srv));

  ros::service::waitForService("/gazebo/spawn_urdf_model", 1.0);
  ASSERT_TRUE(ros::service::exists("/gazebo/spawn_urdf_model", false));

  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel spawn_model_srv;
  spawn_model_srv.request.model_name = "turtlebot";
  spawn_model_srv.request.robot_namespace = "test";

  std::string urdf_file;
  ASSERT_TRUE(ros::param::get("/follower/robot_description", urdf_file));
  spawn_model_srv.request.model_xml = urdf_file;

  geometry_msgs::Pose current_pose;
  current_pose.position.x = 1.5;
  current_pose.position.y = 0.9;
  current_pose.position.z = 0.0;
  current_pose.orientation.w = 1.0;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = 0.0;
  current_pose.orientation.z = 0.0;
  spawn_model_srv.request.initial_pose = current_pose;

  spawn_model_client.call(spawn_model_srv);
  ASSERT_TRUE(spawn_model_srv.response.success);

  ros::service::waitForService("/gazebo/unpause_physics", 1.0);
  ros::ServiceClient unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  ASSERT_TRUE(unpause_physics_client.call(empty_srv));
  ros::spinOnce();
}

TEST(StateDetectionTest, subMultiIRCameras)
{
  ros::NodeHandle nh("");
  ros::Subscriber ir1_sub = nh.subscribe("/guidance/ir_camera_1/scan", 0, callBack);
  ros::Subscriber ir2_sub = nh.subscribe("/guidance/ir_camera_2/scan", 0, callBack2);
  ros::Subscriber ir3_sub = nh.subscribe("/guidance/ir_camera_3/scan", 0, callBack3);

  while (ir1_sub.getNumPublishers() == 0)
    ros::WallDuration(0.1).sleep();
  while (ir2_sub.getNumPublishers() == 0)
    ros::WallDuration(0.1).sleep();
  while (ir3_sub.getNumPublishers() == 0)
    ros::WallDuration(0.1).sleep();

  ros::WallDuration(0.2).sleep();
  ros::spinOnce();
  EXPECT_EQ(cnt, 1) << "Before moving the turtlebot: " << cnt;

  ir1_sub.shutdown();
  ir2_sub.shutdown();
  ir3_sub.shutdown();
}

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "test_state_detect");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
