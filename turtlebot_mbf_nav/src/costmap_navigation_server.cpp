/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  move_base_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/footprint_helper.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_abstract_nav/MoveBaseFlexConfig.h>
#include <actionlib/client/simple_action_client.h>

#include "turtlebot_mbf_nav/costmap_navigation_server.h"

namespace turtlebot_mbf_nav
{


CostmapNavigationServer::CostmapNavigationServer(const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr) :
  AbstractNavigationServer(tf_listener_ptr,
                           mbf_costmap_nav::CostmapPlannerExecution::Ptr(
                                new mbf_costmap_nav::CostmapPlannerExecution(condition_, global_costmap_ptr_)),
                           turtlebot_mbf_nav::CostmapControllerExecution::Ptr(
                                new turtlebot_mbf_nav::CostmapControllerExecution(condition_, tf_listener_ptr,
                                                               local_costmap_ptr_)),
                           mbf_costmap_nav::CostmapRecoveryExecution::Ptr(
                                new mbf_costmap_nav::CostmapRecoveryExecution(condition_, tf_listener_ptr,
                                                             global_costmap_ptr_,
                                                             local_costmap_ptr_))),
    global_costmap_ptr_(new costmap_2d::Costmap2DROS("global_costmap", *tf_listener_ptr_)),
    local_costmap_ptr_(new costmap_2d::Costmap2DROS("local_costmap", *tf_listener_ptr_))
{
  // even if shutdown_costmaps is a dynamically reconfigurable parameter, we
  // need it here to decide weather to start or not the costmaps on starting up
  private_nh_.param("shutdown_costmaps", shutdown_costmaps_, false);

  // initialize costmaps (stopped if shutdown_costmaps is true)
  if (!shutdown_costmaps_)
  {
    local_costmap_active_ = true;
    global_costmap_active_ = true;
  }
  else
  {
    local_costmap_ptr_->stop();
    global_costmap_ptr_->stop();
    local_costmap_active_ = false;
    global_costmap_active_ = false;
  }

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();

  // advertise services and current goal topic
  check_pose_cost_srv_ = private_nh_.advertiseService("check_pose_cost",
                                                      &CostmapNavigationServer::callServiceCheckPoseCost, this);
  clear_costmaps_srv_ = private_nh_.advertiseService("clear_costmaps",
                                                     &CostmapNavigationServer::callServiceClearCostmaps, this);

  current_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

  ros::NodeHandle nh;
  actor_pub_ = nh.advertise<std_msgs::String>("actor_state", 1);

  // dynamic reconfigure server for mbf_costmap_nav configuration; also include abstract server parameters
  dsrv_costmap_ = boost::make_shared<dynamic_reconfigure::Server<mbf_costmap_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_costmap_->setCallback(boost::bind(&CostmapNavigationServer::reconfigure, this, _1, _2));
}

CostmapNavigationServer::~CostmapNavigationServer()
{
  local_costmap_ptr_->stop();
  global_costmap_ptr_->stop();
}

void CostmapNavigationServer::reconfigure(mbf_costmap_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

  // Make sure we have the original configuration the first time we're called, so we can restore it if needed
  if (!setup_reconfigure_)
  {
    default_config_ = config;
    setup_reconfigure_ = true;
  }

  if (config.restore_defaults)
  {
    config = default_config_;
    // if someone sets restore defaults on the parameter server, prevent looping
    config.restore_defaults = false;
  }

  // fill the abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  abstract_config.restore_defaults = config.restore_defaults;
  mbf_abstract_nav::AbstractNavigationServer::reconfigure(abstract_config, level);

  // handle costmap activation reconfiguration here.
  shutdown_costmaps_delay_ = ros::Duration(config.shutdown_costmaps_delay);
  if (shutdown_costmaps_delay_.isZero())
    ROS_WARN("Zero shutdown costmaps delay is not recommended, as it forces us to enable costmaps on each action");

  if (shutdown_costmaps_ && !config.shutdown_costmaps)
  {
    checkActivateCostmaps();
    shutdown_costmaps_ = config.shutdown_costmaps;
  }
  if (!shutdown_costmaps_ && config.shutdown_costmaps)
  {
    shutdown_costmaps_ = config.shutdown_costmaps;
    checkDeactivateCostmaps();
  }

  last_config_ = config;
}

bool CostmapNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                                       mbf_msgs::CheckPose::Response &response)
{
  // selecting the requested costmap
  CostmapPtr costmap;
  std::string costmap_name;
  switch (request.costmap)
  {
    case mbf_msgs::CheckPose::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      costmap_name = "local costmap";
      break;
    case mbf_msgs::CheckPose::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      costmap_name = "global costmap";
      break;
    default:
      ROS_ERROR_STREAM("No valid costmap provided; options are "
                       << mbf_msgs::CheckPose::Request::LOCAL_COSTMAP << ": local costmap, "
                       << mbf_msgs::CheckPose::Request::GLOBAL_COSTMAP << ": global costmap");
      return false;
  }

  // get target pose or current robot pose as x, y, yaw coordinates
  std::string costmap_frame = costmap->getGlobalFrameID();

  geometry_msgs::PoseStamped pose;
  if (request.current_pose)
  {
    if (! mbf_utility::getRobotPose(*tf_listener_ptr_, robot_frame_, costmap_frame, ros::Duration(0.5), pose))
    {
      ROS_ERROR_STREAM("Get robot pose on " << costmap_name << " frame '" << costmap_frame << "' failed");
      return false;
    }
  }
  else
  {
    if (! mbf_utility::transformPose(*tf_listener_ptr_, costmap_frame, request.pose.header.stamp,
                                          ros::Duration(0.5), request.pose, global_frame_, pose))
    {
      ROS_ERROR_STREAM("Transform target pose to " << costmap_name << " frame '" << costmap_frame << "' failed");
      return false;
    }
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf::getYaw(pose.pose.orientation);

  // lock costmap so content doesn't change while adding cell costs
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

  // ensure it's active so cost reflects latest sensor readings
  checkActivateCostmaps();

  // pad raw footprint to the requested safety distance; note that we discard footprint_padding parameter effect
  std::vector<geometry_msgs::Point> footprint = costmap->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, request.safety_dist);

  // use a footprint helper instance to get all the cells totally or partially within footprint polygon
  base_local_planner::FootprintHelper fph;
  std::vector<base_local_planner::Position2DInt> footprint_cells =
    fph.getFootprintCells(Eigen::Vector3f(x, y, yaw), footprint, *costmap->getCostmap(), true);
  response.state = mbf_msgs::CheckPose::Response::FREE;
  if (footprint_cells.empty())
  {
    // no cells within footprint polygon must mean that robot is completely outside of the map
    response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::OUTSIDE));
  }
  else
  {
    // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
    for (int i = 0; i < footprint_cells.size(); ++i)
    {
      unsigned char cost = costmap->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
      switch (cost)
      {
        case costmap_2d::NO_INFORMATION:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::UNKNOWN));
          response.cost += cost;
          break;
        case costmap_2d::LETHAL_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::LETHAL));
          response.cost += cost;
          break;
        case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
          response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::INSCRIBED));
          response.cost += cost;
          break;
        default:response.cost += cost;
          break;
      }
    }
  }

  // Provide some details of the outcome
  switch (response.state)
  {
    case mbf_msgs::CheckPose::Response::OUTSIDE:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is outside the map (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::UNKNOWN:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in unknown space! (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::LETHAL:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in collision! (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::INSCRIBED:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is near an obstacle (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
    case mbf_msgs::CheckPose::Response::FREE:
      ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is free (cost = " << response.cost
                                << "; safety distance = " << request.safety_dist << ")");
      break;
  }

  checkDeactivateCostmaps();

  return true;
}

bool CostmapNavigationServer::callServiceClearCostmaps(std_srvs::Empty::Request &request,
                                                       std_srvs::Empty::Response &response)
{
  local_costmap_ptr_->resetLayers();
  global_costmap_ptr_->resetLayers();
  return true;
}

void CostmapNavigationServer::checkActivateCostmaps()
{
  shutdown_costmaps_timer_.stop();

  // Activate costmaps if we shutdown them when not moving and they are not already active
  if (shutdown_costmaps_ && !local_costmap_active_)
  {
    local_costmap_ptr_->start();
    local_costmap_active_ = true;
    ROS_DEBUG_STREAM("Local costmap activated.");
  }

  if (shutdown_costmaps_ && !global_costmap_active_)
  {
    global_costmap_ptr_->start();
    global_costmap_active_ = true;
    ROS_DEBUG_STREAM("Global costmap activated.");
  }
}

void CostmapNavigationServer::checkDeactivateCostmaps()
{
  if (shutdown_costmaps_ &&
      ((local_costmap_active_ || global_costmap_active_) && !(active_planning_ || active_moving_ || active_recovery_)))
  {
    // Delay costmaps shutdown by shutdown_costmaps_delay so we don't need to enable at each step of a normal
    // navigation sequence, what is terribly inneficient; the timer is stopped on costmaps re-activation and
    // reset after every new call to deactivate
    shutdown_costmaps_timer_ =
      private_nh_.createTimer(shutdown_costmaps_delay_, &CostmapNavigationServer::deactivateCostmaps, this, true);
  }
}

void CostmapNavigationServer::deactivateCostmaps(const ros::TimerEvent &event)
{
  local_costmap_ptr_->stop();
  local_costmap_active_ = false;
  ROS_DEBUG_STREAM("Local costmap deactivated.");
  global_costmap_ptr_->stop();
  global_costmap_active_ = false;
  ROS_DEBUG_STREAM("Global costmap deactivated.");
}

void CostmapNavigationServer::callActionGetPath(const mbf_msgs::GetPathGoalConstPtr &goal)
{
  checkActivateCostmaps();

  // start of abstact get path action call
  ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Start action "  << name_action_get_path);

  mbf_msgs::GetPathResult result;
  geometry_msgs::PoseStamped start_pose, goal_pose;

  result.path.header.seq = path_seq_count_++;
  result.path.header.frame_id = global_frame_;
  result.turning_points.header.seq = path_seq_count_++;
  result.turning_points.header.frame_id = global_frame_;
  goal_pose = goal->target_pose;
  current_goal_pub_.publish(goal_pose);

  double tolerance = goal->tolerance;
  bool use_start_pose = goal->use_start_pose;

  // Try to switch the planner if a special planner is specified in the action goal.
  if(!goal->planner.empty()){

    if(planning_ptr_->switchPlanner(goal->planner))
    {
      ROS_INFO_STREAM("Using the planner \"" << goal->planner << "\".");
    }
    else
    {
      result.outcome = mbf_msgs::GetPathResult::INVALID_PLUGIN;
      result.message = "Could not switch to the planner \"" + goal->planner + "\"!";
      action_server_get_path_ptr_->setAborted(result, result.message);
      return;
    }
  }

  active_planning_ = true;

  if(use_start_pose)
  {
    start_pose = goal->start_pose;
    geometry_msgs::Point p = start_pose.pose.position;
    ROS_INFO_STREAM_NAMED(name_action_get_path, "Use the given start pose ("
                          << p.x << ", " << p.y << ", " << p.z << ").");
  }
  else
  {
    // get the current robot pose
    if (!getRobotPose(start_pose))
    {
      result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
      result.message = "Could not get the current robot pose!";
      action_server_get_path_ptr_->setAborted(result, result.message);
      ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
      return;
    }
    else
    {
      geometry_msgs::Point p = start_pose.pose.position;
      ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Got the current robot pose at ("
                             << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Starting the planning thread.");
  if (!planning_ptr_->startPlanning(start_pose, goal_pose, tolerance))
  {
    result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
    result.message = "Another thread is still planning!";
    action_server_get_path_ptr_->setAborted(result, result.message);
    ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
    return;
  }

  mbf_abstract_nav::AbstractPlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::PoseStamped> plan, global_plan;
  std::vector<geometry_msgs::PoseStamped> turninng_points, global_turninng_points;
  
  double cost;

  int feedback_cnt = 0;

  while (active_planning_ && ros::ok())
  {
    // get the current state of the planning thread
    state_planning_input = planning_ptr_->getState();

    switch (state_planning_input)
    {
      case mbf_abstract_nav::AbstractPlannerExecution::INITIALIZED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot_navigation state: initialized");
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::STARTED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot_navigation state: started");
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::STOPPED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: stopped");
        ROS_WARN_STREAM_NAMED(name_action_get_path, "Planning has been stopped rigorously!");
        result.outcome = mbf_msgs::GetPathResult::STOPPED;
        result.message = "Global planner has been stopped!";
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::CANCELED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: canceled");
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner has been canceled successfully");
        result.path.header.stamp = ros::Time::now();
        result.outcome = mbf_msgs::GetPathResult::CANCELED;
        result.message = "Global planner has been preempted!";
        action_server_get_path_ptr_->setPreempted(result, result.message);
        active_planning_ = false;
        break;

        // in progress
      case mbf_abstract_nav::AbstractPlannerExecution::PLANNING:
        if (planning_ptr_->isPatienceExceeded())
        {
          ROS_INFO_STREAM_NAMED(name_action_get_path, "Global planner patience has been exceeded! "
            << "Cancel planning...");
          if (!planning_ptr_->cancel())
          {
            ROS_WARN_STREAM_THROTTLE_NAMED(2.0, name_action_get_path, "Cancel planning failed or is not supported; "
              "must wait until current plan finish!");
          }
        }
        else
        {
          ROS_DEBUG_THROTTLE_NAMED(2.0, name_action_get_path, "robot navigation state: planning");
        }
        break;

        // found a new plan
      case mbf_abstract_nav::AbstractPlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result.path.header.stamp = ros::Time::now();
        result.turning_points.header.stamp = ros::Time::now();
        plan = planning_ptr_->getPlan();
        publishPath(result.path.poses);

        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: found plan with cost: " << cost);

        if (!transformPlanToGlobalFrame(plan, global_plan))
        {
          result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
          result.message = "Could not transform the plan to the global frame!";

          ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message << " Canceling the action call.");
          action_server_get_path_ptr_->setAborted(result, result.message);
          active_planning_ = false;
          break;
        }

        if (global_plan.empty())
        {
          result.outcome = mbf_msgs::GetPathResult::EMPTY_PATH;
          result.message = "Global planner returned an empty path!";

          ROS_ERROR_STREAM_NAMED(name_action_get_path, result.message);
          action_server_get_path_ptr_->setAborted(result, result.message);
          active_planning_ = false;
          break;
        }

        result.path.poses = global_plan;
        result.cost = planning_ptr_->getCost();
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setSucceeded(result, result.message);

        active_planning_ = false;
        break;

        // no plan found
      case mbf_abstract_nav::AbstractPlannerExecution::NO_PLAN_FOUND:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "robot navigation state: no plan found");
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::MAX_RETRIES:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner reached the maximum number of retries");
        result.outcome = planning_ptr_->getOutcome();
        result.message = planning_ptr_->getMessage();
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::PAT_EXCEEDED:
        ROS_DEBUG_STREAM_NAMED(name_action_get_path, "Global planner exceeded the patience time");
        result.outcome = mbf_msgs::GetPathResult::PAT_EXCEEDED;
        result.message = "Global planner exceeded the patience time";
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
        break;

      case mbf_abstract_nav::AbstractPlannerExecution::INTERNAL_ERROR:
        ROS_FATAL_STREAM_NAMED(name_action_get_path, "Internal error: Unknown error thrown by the plugin!"); // TODO getMessage from planning
        active_recovery_ = false;
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown error thrown by the plugin!";
        action_server_get_path_ptr_->setAborted(result, result.message);
        break;

      default:
        result.outcome = mbf_msgs::GetPathResult::INTERNAL_ERROR;
        result.message = "Internal error: Unknown state in a move base flex planner execution with the number: " + state_planning_input;
        ROS_FATAL_STREAM_NAMED(name_action_get_path, result.message);
        action_server_get_path_ptr_->setAborted(result, result.message);
        active_planning_ = false;
    }

    // if preempt requested while we are planning
    if (action_server_get_path_ptr_->isPreemptRequested()
      && state_planning_input == mbf_abstract_nav::AbstractPlannerExecution::PLANNING)
    {
      if (!planning_ptr_->cancel())
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2.0, name_action_get_path, "Cancel planning failed or is not supported; "
          << "Wait until the current plan finished");
      }
    }

    if (active_planning_)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution thread
      // in order to transfer the results to the controller.
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::milliseconds(500));
    }
  }  // while (active_planning_ && ros::ok())

  if (!active_planning_)
  {
    ROS_DEBUG_STREAM_NAMED(name_action_get_path, "\"GetPath\" action ended properly.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_action_get_path, "\"GetPath\" action has been stopped!");
  }
  // end of abstact get path action call

  checkDeactivateCostmaps();
}

void CostmapNavigationServer::callActionExePath(const mbf_msgs::ExePathGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionExePath(goal);
  checkDeactivateCostmaps();
}

void CostmapNavigationServer::callActionRecovery(const mbf_msgs::RecoveryGoalConstPtr &goal)
{
  checkActivateCostmaps();
  AbstractNavigationServer::callActionRecovery(goal);
  checkDeactivateCostmaps();
}

void CostmapNavigationServer::callActionMoveBase(const mbf_msgs::MoveBaseGoalConstPtr &goal)
{
  ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start action "  << name_action_move_base);

  const geometry_msgs::PoseStamped target_pose = goal->target_pose;

  mbf_msgs::GetPathGoal get_path_goal;
  mbf_msgs::ExePathGoal exe_path_goal;
  mbf_msgs::RecoveryGoal recovery_goal;

  mbf_msgs::MoveBaseResult move_base_result;
  mbf_msgs::GetPathResult get_path_result;
  mbf_msgs::ExePathResult exe_path_result;
  mbf_msgs::RecoveryResult recovery_result;

  for(std::vector<std::string>::const_iterator iter = goal->recovery_behaviors.begin();
      iter != goal->recovery_behaviors.end(); ++iter)
  {
    if(!recovery_ptr_->hasRecoveryBehavior(*iter))
    {
      std::stringstream ss;
      ss << "No recovery behavior with the name \"" << *iter << "\" loaded! ";
      ROS_ERROR_STREAM_NAMED(name_action_move_base, ss.str() << " Please load the behaviors before using them!");
      move_base_result.outcome = mbf_msgs::MoveBaseResult::INVALID_PLUGIN;
      move_base_result.message = ss.str();
      action_server_move_base_ptr_->setAborted(move_base_result, ss.str());
      return;
    }
  }


  get_path_goal.target_pose = target_pose;
  get_path_goal.use_start_pose = false; // use the robot pose

  ros::Duration connection_timeout(1.0);

  // start recovering with the first behavior, use the recovery behaviors from the action request, if specified,
  // otherwise all loaded behaviors.
  std::vector<std::string> recovery_behaviors =
    goal->recovery_behaviors.empty() ? recovery_ptr_->listRecoveryBehaviors() : goal->recovery_behaviors;
  std::vector<std::string>::iterator current_recovery_behavior = recovery_behaviors.begin();

  // get the current robot pose only at the beginning, as exe_path will keep updating it as we move
  if (!getRobotPose(robot_pose_))
  {
    ROS_ERROR_STREAM_NAMED(name_action_move_base, "Could not get the current robot pose!");
    move_base_result.message = "Could not get the current robot pose!";
    move_base_result.outcome = mbf_msgs::MoveBaseResult::TF_ERROR;
    action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
    return;
  }

  enum MoveBaseActionState
  {
    NONE,
    GET_PATH,
    EXE_PATH,
    RECOVERY,
    OSCILLATING
  };

  // wait for server connections
  if (!action_client_get_path_.waitForServer(connection_timeout) ||
      !action_client_exe_path_.waitForServer(connection_timeout) ||
      !action_client_recovery_.waitForServer(connection_timeout))
  {
    ROS_ERROR_STREAM_NAMED(name_action_move_base, "Could not connect to one or more of move_base_flex actions:"
                           << "\"" << name_action_get_path
                           << "\", " << "\"" << name_action_exe_path
                           << "\", " << "\"" << name_action_recovery << "\"!");
    move_base_result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
    move_base_result.message = "Could not connect to the move_base_flex actions!";
    action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
    return;
  }

  active_move_base_ = true;

  // call get_path action server to get a first plan
  action_client_get_path_.sendGoal(get_path_goal);

  //set up the planner's thread
  bool has_new_plan = false;
  boost::mutex planner_mutex;
  boost::condition_variable planner_cond;
  boost::unique_lock<boost::mutex> planner_lock(planner_mutex);
  boost::thread planner_thread(boost::bind(&CostmapNavigationServer::plannerThread, this,
                                           boost::ref(planner_cond), boost::ref(planner_lock),
                                           boost::ref(get_path_goal), boost::ref(get_path_result),
                                           boost::ref(has_new_plan)));

  // init goal states with dummy values;
  actionlib::SimpleClientGoalState get_path_state(actionlib::SimpleClientGoalState::PENDING);
  actionlib::SimpleClientGoalState exe_path_state(actionlib::SimpleClientGoalState::PENDING);
  actionlib::SimpleClientGoalState recovery_state(actionlib::SimpleClientGoalState::PENDING);

  MoveBaseActionState state = GET_PATH;
  MoveBaseActionState recovery_trigger = NONE;

  bool run = true;
  bool preempted = false;
  ros::Duration wait(0.05);

  // we create a navigation-level oscillation detection independent of the exe_path action one,
  // as the later doesn't handle oscillations created by quickly failing repeated plans
  geometry_msgs::PoseStamped oscillation_pose = robot_pose_;
  ros::Time last_oscillation_reset = ros::Time::now();

  std::string type; // recovery behavior type
  std_msgs::String state_msg;
  while (ros::ok() && run)
  {
    bool try_recovery = false;
    switch (state)
    {
      case GET_PATH:
        // To make the actor stop moving except exe_path state
        state_msg.data = "false";
        actor_pub_.publish(state_msg);

        if (!action_client_get_path_.waitForResult(wait))
        { // no result -> action server is still running
          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            action_client_get_path_.cancelGoal();
            preempted = true;
          }
        }
        else
        {
          get_path_state = action_client_get_path_.getState();
          switch (get_path_state.state_)
          {
            case actionlib::SimpleClientGoalState::PENDING:
              // TODO -> not implemented // should not be reached!
              break;

            case actionlib::SimpleClientGoalState::SUCCEEDED:

              get_path_result = *action_client_get_path_.getResult();
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" received a path from \""
                << name_action_get_path << "\": " << get_path_state.getText());

              exe_path_goal.path = get_path_result.path;
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" sends the path to \""
                << name_action_exe_path << "\".");

              // Parse the turning point from global planner to local planner
              exe_path_goal.turning_points = get_path_result.turning_points;

              if (recovery_trigger == GET_PATH)
              {
                ROS_INFO_NAMED(name_action_move_base, "Recovered from planner failure: restart recovery behaviors");
                current_recovery_behavior = recovery_behaviors.begin();
                recovery_trigger = NONE;
              }

              action_client_exe_path_.sendGoal(
                exe_path_goal,
                ActionClientExePath::SimpleDoneCallback(),
                ActionClientExePath::SimpleActiveCallback(),
                boost::bind(&mbf_abstract_nav::AbstractNavigationServer::actionMoveBaseExePathFeedback, this, _1));

              planner_cond.notify_one();

              state = EXE_PATH;
              break;

            case actionlib::SimpleClientGoalState::ABORTED:
              get_path_result = *action_client_get_path_.getResult();

              // copy result from get_path action
              move_base_result.outcome = get_path_result.outcome;
              move_base_result.message = get_path_result.message;
              move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, target_pose));
              move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, target_pose));
              move_base_result.final_pose = robot_pose_;

              if (!recovery_enabled_)
              {
                ROS_WARN_STREAM_NAMED(name_action_move_base, "Recovery behaviors are disabled!");
                ROS_WARN_STREAM_NAMED(name_action_move_base, "Abort the execution of the planner: "
                  << get_path_result.message);
                run = false;
                action_server_move_base_ptr_->setAborted(move_base_result, get_path_state.getText());
                break;
              }
              else if (current_recovery_behavior == recovery_behaviors.end())
              {
                if (recovery_behaviors.empty())
                {
                  ROS_WARN_STREAM_NAMED(name_action_move_base, "No Recovery Behaviors loaded! Abort controlling: "
                    << exe_path_result.message);
                }
                else
                {
                  ROS_WARN_STREAM_NAMED(name_action_move_base, "Executed all available recovery behaviors! "
                    << "Abort planning: " << get_path_result.message);
                }
                run = false;
                action_server_move_base_ptr_->setAborted(move_base_result, get_path_state.getText());
                break;
              }
              else
              {
                recovery_goal.behavior = *current_recovery_behavior;
                recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
                ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start recovery behavior\""
                  << *current_recovery_behavior << "\" of the type \"" << type << "\".");
                action_client_recovery_.sendGoal(recovery_goal);
                recovery_trigger = GET_PATH;
                state = RECOVERY;
              }
              break;

            case actionlib::SimpleClientGoalState::PREEMPTED:
              // the get_path action has been preempted.
              get_path_result = *action_client_get_path_.getResult();

              // copy result from get_path action
              move_base_result.outcome = get_path_result.outcome;
              move_base_result.message = get_path_result.message;
              move_base_result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, target_pose));
              move_base_result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, target_pose));
              move_base_result.final_pose = robot_pose_;
              run = false;
              action_server_move_base_ptr_->setPreempted(move_base_result, get_path_state.getText());
              break;

            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;

            case actionlib::SimpleClientGoalState::LOST:
              // TODO
              break;

            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        break;

      case EXE_PATH:

        // Control the state of the follower
        state_msg.data = "active";
        actor_pub_.publish(state_msg);

        if (has_new_plan)
        {
          ROS_DEBUG("Have new plan; restarting moving action");
          exe_path_goal.path = get_path_result.path;
          action_client_exe_path_.sendGoal(
            exe_path_goal,
            ActionClientExePath::SimpleDoneCallback(),
            ActionClientExePath::SimpleActiveCallback(),
            boost::bind(&mbf_abstract_nav::AbstractNavigationServer::actionMoveBaseExePathFeedback, this, _1));

          has_new_plan = false;
        }
        planner_cond.notify_one();

        if (!action_client_exe_path_.waitForResult(wait))
        {
          // no result -> action server is still running

          if (!oscillation_timeout_.isZero())
          {
            // check if oscillating
            if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance_)
            {
              last_oscillation_reset = ros::Time::now();
              oscillation_pose = robot_pose_;

              if (recovery_trigger == OSCILLATING)
              {
                ROS_INFO_NAMED(name_action_move_base, "Recovered from robot oscillation: restart recovery behaviors");
                current_recovery_behavior = recovery_behaviors.begin();
                recovery_trigger = NONE;
              }
            }
            else if (last_oscillation_reset + oscillation_timeout_ < ros::Time::now())
            {
              ROS_WARN_STREAM_NAMED(name_action_exe_path, "Robot is oscillating for "
                << (ros::Time::now() - last_oscillation_reset).toSec() << "s");
              last_oscillation_reset = ros::Time::now();
              moving_ptr_->stopMoving();

              recovery_trigger = OSCILLATING;
              try_recovery = true;
            }
          }

          if (recovery_trigger == EXE_PATH && moving_ptr_->isMoving())
          {
            ROS_INFO_NAMED(name_action_move_base, "Recovered from controller failure: restart recovery behaviors");
            current_recovery_behavior = recovery_behaviors.begin();
            recovery_trigger = NONE;
          }

          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            action_client_exe_path_.cancelGoal();
            preempted = true;
          }
        }
        else
        {
          exe_path_state = action_client_exe_path_.getState();
          switch (exe_path_state.state_)
          {
            // copy result from get_path action
            exe_path_result = *action_client_exe_path_.getResult();
            move_base_result.outcome = exe_path_result.outcome;
            move_base_result.message = exe_path_result.message;
            move_base_result.dist_to_goal = exe_path_result.dist_to_goal;
            move_base_result.angle_to_goal = exe_path_result.angle_to_goal;
            move_base_result.final_pose = exe_path_result.final_pose;

            case actionlib::SimpleClientGoalState::PENDING:
              //TODO
              break;

            case actionlib::SimpleClientGoalState::SUCCEEDED:
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \""
                << name_action_move_base << "\" received a result from \""
                << name_action_exe_path << "\": " << exe_path_state.getText());
              exe_path_goal.path = get_path_result.path;
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Action \"" << name_action_move_base << "\" succeeded.");

              move_base_result.outcome = mbf_msgs::MoveBaseResult::SUCCESS;
              move_base_result.message = "MoveBase action succeeded!";
              action_server_move_base_ptr_->setSucceeded(move_base_result, move_base_result.message);
              run = false;
              break;

            case actionlib::SimpleClientGoalState::ABORTED:
              switch (exe_path_result.outcome)
              {
                case mbf_msgs::ExePathResult::INVALID_PATH:
                case mbf_msgs::ExePathResult::TF_ERROR:
                case mbf_msgs::ExePathResult::CANCELED:
                case mbf_msgs::ExePathResult::NOT_INITIALIZED:
                case mbf_msgs::ExePathResult::INVALID_PLUGIN:
                case mbf_msgs::ExePathResult::INTERNAL_ERROR:
                  // none of these errors is recoverable
                  run = false;
                  action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
                  break;

                default:
                  // all the rest are, so we start calling the recovery behaviors in sequence
                  recovery_trigger = EXE_PATH;
                  try_recovery = true;
                  break;
              }
              break;

            case actionlib::SimpleClientGoalState::PREEMPTED:
              // action was preempted successfully!
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "The action \""
                << name_action_move_base << "\" was preempted successfully!");
              action_server_move_base_ptr_->setPreempted();
              run = false;
              break;

            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;

            case actionlib::SimpleClientGoalState::LOST:
              // TODO
              break;

            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        if (try_recovery)
        {
          state = RECOVERY;
          if (!recovery_enabled_)
          {
            ROS_WARN_STREAM_NAMED(name_action_move_base, "Recovery behaviors are disabled!");
            ROS_WARN_STREAM_NAMED(name_action_move_base, "Abort the execution of the controller: "
              << exe_path_result.message);
            run = false;
            action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
            break;
          }
          else if (current_recovery_behavior == recovery_behaviors.end())
          {
            if (recovery_behaviors.empty())
            {
              ROS_WARN_STREAM_NAMED(name_action_move_base, "No Recovery Behaviors loaded! Abort controlling: "
                << exe_path_result.message);
            }
            else
            {
              ROS_WARN_STREAM_NAMED(name_action_move_base,
                                    "Executed all available recovery behaviors! Abort controlling: "
                                      << exe_path_result.message);
            }
            run = false;
            action_server_move_base_ptr_->setAborted(move_base_result, exe_path_state.getText());
            break;
          }
          else
          {
            recovery_goal.behavior = *current_recovery_behavior;
            recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
            ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Start recovery behavior\""
              << *current_recovery_behavior << "\" of the type \"" << type << "\".");
            action_client_recovery_.sendGoal(recovery_goal);
            state = RECOVERY;
          }
        }
        break;

      case RECOVERY:

        // To make the actor stop moving except exe_path state
        state_msg.data = "false";
        actor_pub_.publish(state_msg);

        if (!action_client_recovery_.waitForResult(wait))
        {
          if (action_server_move_base_ptr_->isPreemptRequested() && !preempted)
          {
            preempted = true;
            action_client_recovery_.cancelGoal();
          }
        }
        else
        {
          recovery_state = action_client_recovery_.getState();
          switch (recovery_state.state_)
          {
            case actionlib::SimpleClientGoalState::PENDING:
              //TODO
              break;
            case actionlib::SimpleClientGoalState::ABORTED:
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Recovery behavior aborted!");
              recovery_result = *action_client_recovery_.getResult();
              recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "The recovery behavior \""
                << *current_recovery_behavior << "\"" << "of the type \"" << type << "\" failed. ");
              ROS_DEBUG_STREAM("Recovery behavior message: " << recovery_result.message
                                                             << ", outcome: " << recovery_result.outcome);

              current_recovery_behavior++; // use next behavior;
              if (current_recovery_behavior == recovery_behaviors.end())
              {
                ROS_DEBUG_STREAM_NAMED(name_action_move_base,
                                       "All recovery behaviours failed. Abort recovering and abort the move_base action");
                action_server_move_base_ptr_->setAborted(move_base_result, "All recovery behaviors failed.");
                run = false;
              }
              else
              {
                recovery_goal.behavior = *current_recovery_behavior;
                recovery_ptr_->getTypeOfBehavior(*current_recovery_behavior, type);

                ROS_INFO_STREAM_NAMED(name_action_move_base, "Run the next recovery behavior\""
                  << *current_recovery_behavior << "\" of the type \"" << type << "\".");
                action_client_recovery_.sendGoal(recovery_goal);
              }
              break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
              recovery_result = *action_client_recovery_.getResult();
              //go to planning state
              ROS_DEBUG_STREAM_NAMED(name_action_move_base, "Execution of the recovery behavior \""
                << *current_recovery_behavior << "\" succeeded!");
              ROS_DEBUG_STREAM_NAMED(name_action_move_base,
                                     "Try planning again and increment the current recovery behavior in the list.");
              state = GET_PATH;
              current_recovery_behavior++; // use next behavior, the next time;
              action_client_get_path_.sendGoal(get_path_goal);
              break;
            case actionlib::SimpleClientGoalState::PREEMPTED:
              run = false;
              action_server_move_base_ptr_->setPreempted();
              break;
            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "The states RECALLED and REJECTED are not implemented in the SimpleActionServer!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
            case actionlib::SimpleClientGoalState::LOST:
            default:
              ROS_FATAL_STREAM_NAMED(name_action_move_base,
                                     "Reached unreachable case! Unknown SimpleActionServer state!");
              run = false;
              action_server_move_base_ptr_->setAborted();
              break;
          }
        }

        break;

      default:
        move_base_result.outcome = mbf_msgs::MoveBaseResult::INTERNAL_ERROR;
        move_base_result.message = "Reached a undefined case! Please report the bug!";
        action_server_move_base_ptr_->setAborted(move_base_result, move_base_result.message);
        ROS_FATAL_STREAM_NAMED(name_action_move_base, move_base_result.message);
        break;
    }
  }

  planner_thread.interrupt();
  planner_thread.join();

  active_move_base_ = false;
}

void CostmapNavigationServer::plannerThread(boost::condition_variable &cond, boost::unique_lock<boost::mutex> &lock,
                                             const mbf_msgs::GetPathGoal &goal, mbf_msgs::GetPathResult &result,
                                             bool &has_new_plan)
{
  if (planning_ptr_->getFrequency() <= 0.0)
    return;

  ros::Rate rate(planning_ptr_->getFrequency());  // TODO: will ignore dyn. reconf. until next run

  while (ros::ok())
  {
    ROS_DEBUG("Planner thread waiting...");
    cond.wait(lock);
    ROS_DEBUG("Planner thread started!");

    // keep calling get_path action server at planner_frequency Hz to get updated plans
    action_client_get_path_.sendGoal(goal);
    action_client_get_path_.waitForResult();
    result = *action_client_get_path_.getResult();
    if (result.outcome < 10)
      has_new_plan = true;
    rate.sleep();
  }
}

} /* namespace mbf_costmap_nav */
