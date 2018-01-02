#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>

#include <gtest/gtest.h>

bool inTolerance(const double &val,
                 const double &ref,
                 const double &range)
{
  return val < (ref+range) && val > (ref-range) ? true : false;
}

/// Test spawning the robot using spawn_urdf_model service
/// Also, pause the physics (ODE) before the calling the service
TEST(SpawnRobotTest, spawnURDFService)
{
  ros::NodeHandle nh("~");
  ros::service::waitForService("/gazebo/pause_physics", 1.0);
  ros::ServiceClient pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty empty_srv;
  ASSERT_TRUE(pause_physics_client.call(empty_srv));

  ros::service::waitForService("/gazebo/spawn_urdf_model", 1.0);
  ASSERT_TRUE(ros::service::exists("/gazebo/spawn_urdf_model", false));

  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel spawn_model_srv;
  spawn_model_srv.request.model_name = "turtlebot";
  spawn_model_srv.request.robot_namespace = "";

  std::string urdf_file;
  ASSERT_TRUE(ros::param::get("robot_description", urdf_file));
  spawn_model_srv.request.model_xml = urdf_file;

  geometry_msgs::Pose current_pose;
  current_pose.position.x = 1.0;
  current_pose.position.y = 1.0;
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
}

/// Check the robot is in the scene using get_world_propoerties service
/// Then check the robot is within tolerance
TEST(SpawnRobotTest, checkWorldRobot)
{
  ros::NodeHandle nh("~");
  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_world_properties"));
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  gazebo_msgs::GetWorldProperties world_properties;
  ASSERT_TRUE(check_model_client.call(world_properties));
  std::vector<std::string> model_names = world_properties.response.model_names;
  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "turtlebot") != model_names.end());

  ASSERT_TRUE(ros::service::exists("/gazebo/get_model_state", true));
  ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState get_model_state_srv;
  get_model_state_srv.request.model_name = "turtlebot";
  get_model_state_client.call(get_model_state_srv);
  EXPECT_TRUE(inTolerance(get_model_state_srv.response.pose.position.x, 1.0, 0.1));
  EXPECT_TRUE(inTolerance(get_model_state_srv.response.pose.position.y, 1.0, 0.1));
}


/// Check the spawned robot has IR Led link
TEST(SpawnRobotTest, checkIRLedLink)
{
  ros::NodeHandle nh("~");
  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_model_properties"));
  ros::ServiceClient check_model_srv = nh.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
  gazebo_msgs::GetModelProperties model_properties;
  model_properties.request.model_name = "turtlebot";
  ASSERT_TRUE(check_model_srv.call(model_properties));

  std::vector<std::string> body_names = model_properties.response.body_names;
  std::vector<std::string> joint_names = model_properties.response.joint_names;
  EXPECT_TRUE(std::find(body_names.begin(), body_names.end(), "ir_led_link") != body_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "ir_led_joint") != joint_names.end());
}

/// Delete the robot after the test is finished
/// TODO: Currently the service sometimes return not response, still needs to fix.
TEST(SpawnRobotTest, deleteRobot)
{
  ros::NodeHandle nh("~");
  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_model_properties"));
  ros::ServiceClient delete_model_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  gazebo_msgs::DeleteModel delete_model;

  delete_model.request.model_name = "turtlebot";
  delete_model_client.call(delete_model);
  EXPECT_TRUE(delete_model.response.success) << delete_model.response.status_message;
  ros::Duration(0.1).sleep();
}

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "test_state_detect");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
