cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_mbf_nav)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  mbf_abstract_nav
  mbf_costmap_core
  mbf_costmap_nav
  mbf_msgs
  nav_core
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  rviz_visual_tools
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

set(TURTLEBOT_MBF_SERVER_LIB turtlebot_mbf_server)
set(TURTLEBOT_MBF_SERVER_NODE turtlebot_mbf_nav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${TURTLEBOT_MBF_SERVER_LIB}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  mbf_abstract_nav
  mbf_costmap_core
  mbf_costmap_nav
  mbf_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
  rviz_visual_tools
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${TURTLEBOT_MBF_SERVER_LIB}
  src/costmap_navigation_server.cpp
)
target_link_libraries(${TURTLEBOT_MBF_SERVER_LIB}
  ${MBF_NAV_CORE_WRAPPER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(${TURTLEBOT_MBF_SERVER_LIB} ${catkin_EXPORTED_TARGETS})

add_executable(${TURTLEBOT_MBF_SERVER_NODE} src/move_base_server_node.cpp)
add_dependencies(${TURTLEBOT_MBF_SERVER_NODE} ${TURTLEBOT_MBF_SERVER_LIB})
target_link_libraries(${TURTLEBOT_MBF_SERVER_NODE}
  ${TURTLEBOT_MBF_SERVER_LIB}
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/move_base_legacy_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
