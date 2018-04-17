#include <nav_core_wrapper/wrapper_local_planner.h>
#include "turtlebot_mbf_nav//costmap_controller_execution.h"

namespace turtlebot_mbf_nav
{

CostmapControllerExecution::CostmapControllerExecution(
    boost::condition_variable &condition, const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
    CostmapPtr &costmap_ptr) :
    AbstractControllerExecution(condition, tf_listener_ptr),
    costmap_ptr_(costmap_ptr)
{
}

CostmapControllerExecution::~CostmapControllerExecution()
{
}

mbf_abstract_core::AbstractController::Ptr CostmapControllerExecution::loadControllerPlugin(const std::string& controller_type)
{
  static pluginlib::ClassLoader<mbf_costmap_core::CostmapController>
      class_loader("mbf_costmap_core", "mbf_costmap_core::CostmapController");
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  // try to load and init local planner
  ROS_DEBUG("Load local planner plugin.");
  try
  {
    controller_ptr = class_loader.createInstance(controller_type);
    controller_name_ = class_loader.getName(controller_type);
    ROS_INFO_STREAM("MBF_core-based local planner plugin " << controller_name_ << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    ROS_INFO_STREAM("Failed to load the " << controller_type << " local planner as a mbf_abstract_core-based plugin;"
                     << "  we will retry to load as a nav_core-based plugin. Exception: " << ex.what());
    try
    {
      // For plugins still based on old nav_core API, we load them and pass to a new MBF API that will act as wrapper
      static pluginlib::ClassLoader<nav_core::BaseLocalPlanner>
          nav_core_class_loader("nav_core", "nav_core::BaseLocalPlanner");
      boost::shared_ptr<nav_core::BaseLocalPlanner> nav_core_controller_ptr
          = nav_core_class_loader.createInstance(controller_type);
      controller_ptr = boost::make_shared<mbf_nav_core_wrapper::WrapperLocalPlanner>(nav_core_controller_ptr);
      controller_name_ = nav_core_class_loader.getName(controller_type);
      ROS_INFO_STREAM("Nav_core-based local planner plugin " << controller_name_ << " loaded");
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL_STREAM("Failed to load the " << controller_type
          << " local planner, are you sure it's properly registered"
          << " and that the containing library is built? Exception: " << ex.what());
    }
  }
  return controller_ptr;
}

void CostmapControllerExecution::initPlugin()
{
  ROS_INFO_STREAM("Initialize controller \"" << controller_name_ << "\".");

  if (!tf_listener_ptr)
  {
    ROS_ERROR_STREAM("The tf listener pointer has not been initialized!");
    exit(1);
  }

  if (!costmap_ptr_)
  {
    ROS_ERROR_STREAM("The costmap pointer has not been initialized!");
    exit(1);
  }

  ros::NodeHandle private_nh("~");
  private_nh.param("controller_lock_costmap", lock_costmap_, true);

  mbf_costmap_core::CostmapController::Ptr controller_ptr
      = boost::static_pointer_cast<mbf_costmap_core::CostmapController>(controller_);
  controller_ptr->initialize(controller_name_, tf_listener_ptr.get(), costmap_ptr_.get());
  ROS_INFO_STREAM("Controller plugin \"" << controller_name_ << "\" initialized.");
}

uint32_t CostmapControllerExecution::computeVelocityCmd(const geometry_msgs::PoseStamped& robot_pose,
                                                        const geometry_msgs::TwistStamped& robot_velocity,
                                                        geometry_msgs::TwistStamped& vel_cmd,
                                                        std::string& message)
{
  // Lock the costmap while planning, but following issue #4, we allow to move the responsibility to the planner itself
  if (lock_costmap_)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
  }
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

} /* namespace mbf_costmap_nav */
