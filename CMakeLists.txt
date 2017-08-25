cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guidance)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rviz_visual_tools
)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    rviz_visual_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

SET(follower_tracker_files
  src/follower_tracker.cpp
  include/follower_tracker.h
)

add_library(follower_tracker ${follower_tracker_files})
target_link_libraries(follower_tracker
    ${catkin_LIBRARIES}
)
add_dependencies(follower_tracker ${catkin_EXPORTED_TARGETS})

add_executable(follower_tracker_node src/follower_tracker_node.cpp)
target_link_libraries(follower_tracker_node follower_tracker)
