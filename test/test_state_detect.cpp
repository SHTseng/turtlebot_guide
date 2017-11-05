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

class StateDetect : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
  }

  ros::NodeHandle nh_;
  ros::Subscriber cam_sub_;
  bool has_new_image_;
  int model_cnt_;
  ros::Time image_stamp_;
public:
  void imageCallback(const gazebo_ir_camera_plugin::IRCameraConstPtr& msg)
  {
    image_stamp_ = msg->header.stamp;
    model_cnt_ = msg->azimuthal_angles.size();
    has_new_image_ = true;
}
};

bool has_new_image = false;
int cnt = 0;

void callBack(const gazebo_ir_camera_plugin::IRCameraConstPtr &msg)
{
  cnt = msg->azimuthal_angles.size();
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
  current_pose.position.x = 1.5;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.w = 1.0;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = 0.0;
  current_pose.orientation.z = 0.0;
  spawn_model_srv.request.initial_pose = current_pose;

  ASSERT_TRUE(spawn_model_client.call(spawn_model_srv));
  ASSERT_TRUE(spawn_model_srv.response.success);

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
  ASSERT_TRUE(ros::service::waitForService("/gazebo/get_world_properties"));
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  gazebo_msgs::GetWorldProperties world_properties;
  ASSERT_TRUE(check_model_client.call(world_properties));
  std::vector<std::string> model_names = world_properties.response.model_names;
  ASSERT_TRUE(std::find(model_names.begin(), model_names.end(), "follower") != model_names.end());
//  helper h;
  ros::Subscriber ir_sub_ = nh.subscribe("/guidance/ir_camera_1/scan", 0, callBack);
  ros::Publisher pub = nh.advertise<gazebo_ir_camera_plugin::IRCamera>("/ir_camera_1/scan", 0);
  while (ir_sub_.getNumPublishers() == 0)
  {
    ros::Duration(0.01).sleep();
  }
  ASSERT_EQ(ir_sub_.getNumPublishers(), 1) << "IR Camera publisher does not exist";
//  ASSERT_EQ(pub.getNumSubscribers(), 1);

  gazebo_ir_camera_plugin::IRCamera msg;
  msg.azimuthal_angles.push_back(0.0);
  pub.publish(msg);

  ros::Duration(0.5).sleep();
  ros::spinOnce();
  EXPECT_EQ(cnt, 1U) << "Robot is not in the IR camera frame";
}

TEST(StateDetectTest, endTest)
{
  ros::shutdown();
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
  ros::init(argc, argv, "test_state_detect");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
