cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_follower_tracker)
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS core filters io segmentation)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  tf
  eigen_conversions
  nodelet
  std_msgs
  sensor_msgs
  nav_msgs
  spencer_tracking_msgs
  turtlebot_guide_msgs
  image_transport 
  cv_bridge  
  spencer_diagnostics
  rwth_perception_people_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    tf
    eigen_conversions
    nodelet
    std_msgs
    sensor_msgs
    nav_msgs
    spencer_tracking_msgs
    turtlebot_guide_msgs
    image_transport 
    cv_bridge  
    spencer_diagnostics
    rwth_perception_people_msgs
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
target_link_libraries(follower_tracker_nodelet pcl_people_detection_ros ${catkin_LIBRARIES})

add_executable(upper_body_detector src/upper_body_detector.cpp
  src/upper_body_detector/Camera.cpp src/upper_body_detector/ConfigFile.cpp src/upper_body_detector/Globals.cpp
  src/upper_body_detector/KConnectedComponentLabeler.cpp
  src/upper_body_detector/Math.cpp src/upper_body_detector/pointcloud.cpp src/upper_body_detector/ROI.cpp
  src/upper_body_detector/detector.cpp src/upper_body_detector/AncillaryMethods.cpp
  src/upper_body_detector/Hypo.cpp src/upper_body_detector/FrameInlier.cpp)
target_link_libraries(upper_body_detector ${catkin_LIBRARIES})
 
add_executable(follower_state_monitor src/follower_state_monitor.cpp)
target_link_libraries(follower_state_monitor ${catkin_LIBRARIES})
add_dependencies(follower_state_monitor turtlebot_guide_msgs_generate_messages_cpp)
