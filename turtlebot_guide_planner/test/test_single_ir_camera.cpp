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
  current_pose.position.y = 0.1;
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

TEST(StateDetectionTest, subSingleIRCamera)
{
  ros::NodeHandle nh("");
  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_world_properties"));
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  gazebo_msgs::GetWorldProperties world_properties;
  ASSERT_TRUE(check_model_client.call(world_properties));
  std::vector<std::string> model_names = world_properties.response.model_names;
  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "turtlebot_follower") != model_names.end());
//  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "turtlebot") != model_names.end());

  ros::Subscriber ir_sub_ = nh.subscribe("/guidance/ir_camera_1/scan", 0, callBack);
  while (ir_sub_.getNumPublishers() == 0)
  {
    ros::Duration(0.01).sleep();
  }
  ASSERT_EQ(ir_sub_.getNumPublishers(), 1) << "IR Camera publisher does not exist";

  // Essential sleep to make the subscriber catch the data
  ros::WallDuration(0.2).sleep();
  cnt = 0;
  ros::spinOnce();
  EXPECT_EQ(cnt, 2) << cnt << " Robots are not in the IR camera frame";
}

TEST(StateDetectionTest, moveRobot)
{
  ros::NodeHandle nh("");
  ros::service::waitForService("/gazebo/get_model_state", 1.0);
  ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState get_model_state_srv;
  get_model_state_srv.request.model_name = "turtlebot_follower";
  get_model_state_client.call(get_model_state_srv);
  EXPECT_TRUE(inTolerance(get_model_state_srv.response.pose.position.x, 1.4, 0.1));

  ros::Subscriber ir_sub = nh.subscribe("/guidance/ir_camera_1/scan", 0, callBack);
  while (ir_sub.getNumPublishers() == 0)
    ros::WallDuration(0.1).sleep();

  // Essential sleep to make the subscriber catch the data
  ros::WallDuration(0.2).sleep();
  int prev_cnt = cnt;
  cnt = 0;
  ros::spinOnce();
  EXPECT_EQ(cnt, prev_cnt) << "Before moving the turtlebot: " << cnt;

  ros::service::waitForService("/gazebo/set_model_state", 1.0);
  ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState set_model_state_srv;
  gazebo_msgs::ModelState model_state_msg;
  model_state_msg.model_name = "turtlebot_follower";
  model_state_msg.pose.position.x = 2.5;
  set_model_state_srv.request.model_state = model_state_msg;
  set_model_state_client.call(set_model_state_srv);

  ros::WallDuration(0.2).sleep();
  cnt = 0;
  ros::spinOnce();
  EXPECT_EQ(cnt, 1) << "After moving the turtlebot: " << cnt;

  get_model_state_srv.request.model_name = "turtlebot_follower";
  ASSERT_TRUE(get_model_state_client.call(get_model_state_srv));
  EXPECT_TRUE(inTolerance(get_model_state_srv.response.pose.position.x, 2.5, 0.1));
}

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "test_state_detect");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
