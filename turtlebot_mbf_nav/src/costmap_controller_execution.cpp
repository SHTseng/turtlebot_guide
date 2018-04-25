#include <nav_core_wrapper/wrapper_local_planner.h>
#include "turtlebot_mbf_nav/costmap_controller_execution.h"

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

bool CostmapControllerExecution::initPlugin(
    const std::string& name, 
    const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  ROS_INFO_STREAM("Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr)
  {
    ROS_ERROR_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!costmap_ptr_)
  {
    ROS_ERROR_STREAM("The costmap pointer has not been initialized!");
    return false;
  }

  ros::NodeHandle private_nh("~");
  private_nh.param("controller_lock_costmap", lock_costmap_, true);

  mbf_costmap_core::CostmapController::Ptr costmap_controller_ptr
      = boost::static_pointer_cast<mbf_costmap_core::CostmapController>(controller_ptr);
  costmap_controller_ptr->initialize(name, tf_listener_ptr.get(), costmap_ptr_.get());
  ROS_INFO_STREAM("Controller plugin \"" << name << "\" initialized.");
  return true;
}

void CostmapControllerExecution::run()
{
  start_time_ = ros::Time::now();

  // init plan
  std::vector<geometry_msgs::PoseStamped> plan;
  std::vector<geometry_msgs::PoseStamped> turning_points;
  int nearest_turning_pt_index = 0;
  if (!hasNewPlan())
  {
    setState(NO_PLAN);
    moving_ = false;
    ROS_ERROR("robot navigation moving has no plan!");
  }

  int retries = 0;
  int seq = 0;

  try
  {
    while (moving_ && ros::ok())
    {
      boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

      boost::chrono::thread_clock::time_point loop_start_time = boost::chrono::thread_clock::now();

      // update plan dynamically
      if (hasNewPlan())
      {
        plan = getNewPlan();
        
        // check if plan is empty
        if (plan.empty())
        {
          setState(EMPTY_PLAN);
          condition_.notify_all();
          moving_ = false;
          return;
        }

        // check if plan could be set
        if(!controller_->setPlan(plan))
        {
          setState(INVALID_PLAN);
          condition_.notify_all();
          moving_ = false;
          return;
        }

      }

      // compute robot pose and store it in robot_pose_
      computeRobotPose();

      // ask planner if the goal is reached
      if (reachedGoalCheck())
      {
        setState(ARRIVED_GOAL);
        // goal reached, tell it the controller
        condition_.notify_all();
        moving_ = false;
        // if not, keep moving
      }
      else
      {
        setState(PLANNING);
        
        // obtain turning point from global planner
        turning_points = getTurningPoints();
        nearest_turning_pt_index = nearestTurningPoint(robot_pose_, turning_points);
        // if (nearest_turning_pt_index != -1) 
        //   ROS_INFO_STREAM(nearest_turning_pt_index);

        // save time and call the plugin
        lct_mtx_.lock();
        last_call_time_ = ros::Time::now();
        lct_mtx_.unlock();

        // call plugin to compute the next velocity command
        geometry_msgs::TwistStamped cmd_vel_stamped;
        outcome_ = computeVelocityCmd(cmd_vel_stamped, message_);

        if (outcome_ < 10)
        {
          // set stamped values: frame id, time stamp and sequence number
          cmd_vel_stamped.header.seq = seq++;
          setVelocityCmd(cmd_vel_stamped);
          setState(GOT_LOCAL_CMD);
          vel_pub_.publish(cmd_vel_stamped.twist);
          condition_.notify_all();
          retries = 0;
        }
        else
        {
          if (++retries > max_retries_)
          {
            setState(MAX_RETRIES);
            moving_ = false;
            condition_.notify_all();
          }
          else if (ros::Time::now() - getLastValidCmdVelTime() > patience_
              && ros::Time::now() - start_time_ > patience_)  // why not isPatienceExceeded() ?
          {
            setState(PAT_EXCEEDED);
            moving_ = false;
            condition_.notify_all();
          }
          else
          {
            setState(NO_LOCAL_CMD); // useful for server feedback
            condition_.notify_all();
          }
          // could not compute a valid velocity command -> stop moving the robot
          publishZeroVelocity(); // command the robot to stop
        }
      }

      boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
      boost::chrono::microseconds execution_duration =
          boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - loop_start_time);
      boost::chrono::microseconds sleep_time = calling_duration_ - execution_duration;
      if (moving_ && ros::ok())
      {
        if (sleep_time > boost::chrono::microseconds(0))
        {
          // interruption point
          boost::this_thread::sleep_for(sleep_time);
        }
        else
        {
          ROS_WARN_THROTTLE(1.0, "Calculation needs too much time to stay in the moving frequency!");
        }
      }
    }
  }
  catch (const boost::thread_interrupted &ex)
  {
    // Controller thread interrupted; in most cases we have started a new plan
    // Can also be that robot is oscillating or we have exceeded planner patience
    ROS_DEBUG_STREAM("Controller thread interrupted!");
    // publishZeroVelocity();  TODO comment this makes sense for continuous replanning
    setState(STOPPED);
    condition_.notify_all();
    moving_ = false;
  }
  catch (...){
    message_ = "Unknown error occurred: " + boost::current_exception_diagnostic_information();
    ROS_FATAL_STREAM(message_);
    setState(INTERNAL_ERROR);
  }
}

int CostmapControllerExecution::nearestTurningPoint(
    const geometry_msgs::PoseStamped& robot_pose, 
    const std::vector<geometry_msgs::PoseStamped>& points)
{
  if (points.empty()){
    return -1;
  }

  for (int i = 0; i < points.size(); i++){
    if (hypot(robot_pose.pose.position.x-points[i].pose.position.x, 
              robot_pose.pose.position.y-points[i].pose.position.y) < 0.5){
      return i;
    }
  }
  return -1;
}

std::vector<geometry_msgs::PoseStamped> CostmapControllerExecution::getTurningPoints()
{
  boost::lock_guard<boost::mutex> guard(plan_mtx_);
  return turning_points_;
}

// void CostmapControllerExecution::setTurningPoints(const std::vector<geometry_msgs::PoseStamped>& points)
// {
//   turning_points_ = points;
// }

// uint32_t CostmapControllerExecution::computeVelocityCmd(const geometry_msgs::PoseStamped& robot_pose,
//                                                         const geometry_msgs::TwistStamped& robot_velocity,
//                                                         geometry_msgs::TwistStamped& vel_cmd,
//                                                         std::string& message)
// {
//   // Lock the costmap while planning, but following issue #4, we allow to move the responsibility to the planner itself
//   if (lock_costmap_)
//   {
//     boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
//     return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
//   }
//   return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
// }

} /* namespace mbf_costmap_nav */