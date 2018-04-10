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
  nodelet
  sensor_msgs
  spencer_tracking_msgs
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
    nodelet
    sensor_msgs
    spencer_tracking_msgs
  DEPENDS
    Boost
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

add_library(pcl_people_detection_ros src/pcl_people_detection_ros.cpp src/pcl_people_detection.cpp)
target_link_libraries(pcl_people_detection_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

add_library(follower_tracker_nodelet src/follower_tracker_nodelet.cpp)
target_link_libraries(follower_tracker_nodelet  pcl_people_detection_ros ${catkin_LIBRARIES})