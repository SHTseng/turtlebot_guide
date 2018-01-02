cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_follower_tracker)
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS core filters io segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  tf
  eigen_conversions
  gazebo_ir_camera_plugin
  nodelet
  sensor_msgs
  rviz_visual_tools
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pcl_conversions
    pcl_ros
    tf
    eigen_conversions
    gazebo_ir_camera_plugin
    nodelet
    sensor_msgs
    rviz_visual_tools
  DEPENDS
    Boost
    Eigen3
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

SET(follower_tracker_files
  src/follower_tracker.cpp
  src/mesh_model_loader.cpp
  include/turtlebot_follower_tracker/follower_tracker.h
  include/turtlebot_follower_tracker/mesh_model_loader.h
)

add_library(follower_tracker ${follower_tracker_files})
target_link_libraries(follower_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

add_library(FollowerTrackerROS src/follower_tracker_ros.cpp include/turtlebot_follower_tracker/follower_tracker_ros.h)
target_link_libraries(FollowerTrackerROS follower_tracker ${catkin_LIBRARIES})

add_library(FollowerTrackerNodelet src/follower_tracker_nodelet.cpp)
target_link_libraries(FollowerTrackerNodelet FollowerTrackerROS ${catkin_LIBRARIES})

add_executable(load_model_test src/load_model_test.cpp)
target_link_libraries(load_model_test follower_tracker ${catkin_LIBRARIES})

add_executable(capture_scene util/capture_scene.cpp)
target_link_libraries(capture_scene ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(follower_tracker_node src/follower_tracker_node.cpp)
#target_link_libraries(follower_tracker_node follower_tracker ${catkin_LIBRARIES})
