#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <gtest/gtest.h>

#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>

#include <gazebo_ir_camera_plugin/IRCamera.h>

#include <tinyxml.h>

struct helper
{
  helper(){}

  void cb(const gazebo_ir_camera_plugin::IRCamera &msg)
  {
    msg_ = msg;
  }

  gazebo_ir_camera_plugin::IRCamera msg_;
};

gazebo_ir_camera_plugin::IRCamera ir_camera_msg_;

void callBack(const gazebo_ir_camera_plugin::IRCamera &msg)
{
  ir_camera_msg_ = msg;
}

TEST(StateDetectTest, spawnRobot)
{
  ros::NodeHandle nh("");
  ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
  ASSERT_TRUE(ros::service::exists("/gazebo/spawn_urdf_model", false));

  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel spawn_model_srv;
  spawn_model_srv.request.model_name = "follower";
  spawn_model_srv.request.robot_namespace = "";

  TiXmlDocument xml_in(ros::package::getPath("turtlebot_guidance")+"/robot/turtlebot_follower.urdf");
  ASSERT_TRUE(xml_in.LoadFile());
  std::ostringstream stream;
  stream << xml_in;
  spawn_model_srv.request.model_xml = stream.str(); // load xml file
  ROS_DEBUG_NAMED("spawn_box", "XML string: %s",stream.str().c_str());

  geometry_msgs::Pose current_pose;
  current_pose.position.x = 1.6;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.w = 1.0;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = 0.0;
  current_pose.orientation.z = 0.0;
  spawn_model_srv.request.initial_pose = current_pose;

  ASSERT_TRUE(spawn_model_client.call(spawn_model_srv));

  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_world_properties"));
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  gazebo_msgs::GetWorldProperties world_properties;
  ASSERT_TRUE(check_model_client.call(world_properties));
  std::vector<std::string> model_names = world_properties.response.model_names;
  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "follower") != model_names.end());

  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_model_properties"));
  ros::ServiceClient check_model_srv = nh.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
  gazebo_msgs::GetModelProperties model_properties;
  model_properties.request.model_name = "follower";
  ASSERT_TRUE(check_model_srv.call(model_properties));

  std::vector<std::string> body_names = model_properties.response.body_names;
  std::vector<std::string> joint_names = model_properties.response.joint_names;
  EXPECT_TRUE(std::find(body_names.begin(), body_names.end(), "ir_led_link") != body_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "ir_led_joint") != joint_names.end());

  SUCCEED();
}

TEST(StateDetectTest, subscribeIRTopic)
{
  ros::NodeHandle nh;
  helper h;
  ros::Subscriber ir_sub_ = nh.subscribe("/guidance/ir_camera_1/scan", 1, &helper::cb, &h);
  while (ir_sub_.getNumPublishers() == 0)
  {
    ros::spinOnce();
  }
  EXPECT_EQ(ir_sub_.getNumPublishers(), 1) << "IR Camera publisher does not exist";
  ASSERT_TRUE(ros::getGlobalCallbackQueue()->isEmpty());

  EXPECT_EQ(h.msg_.azimuthal_angles.size(), 1) << "Robot is not in the IR camera frame";
  ir_sub_.shutdown();
}

//TEST(StateDetectTest, deleteRobot)
//{
//  ros::NodeHandle nh("");
//  ASSERT_TRUE(ros::service::exists("/gazebo/delete_model", false));
//  // make the service call to delete model
//  ASSERT_TRUE(ros::service::waitForService("/gazebo/delete_model"));
//  ros::ServiceClient delete_model_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
//  gazebo_msgs::DeleteModel delete_model;

//  delete_model.request.model_name = "follower";
//  delete_model_client.call(delete_model);
//  ASSERT_TRUE(delete_model.response.success);

//  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_world_properties"));
//  ros::ServiceClient check_model_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
//  gazebo_msgs::GetWorldProperties world_properties;
//  ASSERT_TRUE(check_model_client.call(world_properties));
//  std::vector<std::string> model_names = world_properties.response.model_names;
//  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "follower") == model_names.end());

//  SUCCEED();
//}


int main (int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_state_detect");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();

  //  ros::AsyncSpinner spinner(1);
//  spinner.start();
//  int ret = RUN_ALL_TESTS();
//  spinner.stop();
//  ros::shutdown();
//  return ret;
}
