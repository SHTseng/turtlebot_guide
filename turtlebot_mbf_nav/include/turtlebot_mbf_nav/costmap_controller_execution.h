#ifndef COSTMAP_CONTROLLER_EXECUTION_H
#define COSTMAP_CONTROLLER_EXECUTION_H

#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <mbf_abstract_nav/abstract_controller_execution.h>

namespace turtlebot_mbf_nav
{

class CostmapControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:

  typedef boost::shared_ptr<costmap_2d::Costmap2DROS> CostmapPtr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param costmap_ptr Shared pointer to the costmap.
   */
  CostmapControllerExecution(boost::condition_variable &condition,
                              const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                              CostmapPtr &costmap_ptr);

  /**
   * @brief Destructor
   */
  virtual ~CostmapControllerExecution();

protected:

  /**
   * @brief Request plugin for a new velocity command. We override this method so we can lock the local costmap
   *        before calling the planner.
   * @param vel_cmd_stamped current velocity command
   */

  // virtual uint32_t computeVelocityCmd(
  //     const geometry_msgs::PoseStamped& robot_pose,
  //     const geometry_msgs::TwistStamped& robot_velocity,
  //     geometry_msgs::TwistStamped& vel_cmd,
  //     std::string& message);

private:

  /**
   * @brief The main run method, a thread will execute this method. It contains the main controller execution loop.
   */
  virtual void run();

  std::vector<geometry_msgs::PoseStamped> getTurningPoints();
  
  int nearestTurningPoint(const geometry_msgs::PoseStamped& robot_pose, const std::vector<geometry_msgs::PoseStamped>& points);

  std::vector<geometry_msgs::PoseStamped> turning_points_;
  
  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
   *         an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type);

  /**
   * @brief Initializes the local planner plugin with its name, a pointer to the TransformListener
   *        and pointer to the costmap
   */
  virtual bool initPlugin(const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr);

  //! costmap for 2d navigation planning
  CostmapPtr &costmap_ptr_;

  //! Whether to lock costmap before calling the controller (see issue #4 for details)
  bool lock_costmap_;

  //! name of the controller plugin assigned by the class loader
  std::string controller_name_;
};

}

#endif // COSTMAP_CONTROLLER_EXECUTION_H
