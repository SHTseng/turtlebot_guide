cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guidance)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rviz_visual_tools
  gazebo_ir_camera_plugin
  move_base_msgs
  actionlib
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
    gazebo_ir_camera_plugin
    move_base_msgs
    actionlib
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

SET(follower_tracker_files
  src/follower_tracker.cpp
  include/turtlebot_guidance/follower_tracker.h
)

add_library(follower_tracker ${follower_tracker_files})
target_link_libraries(follower_tracker
    ${catkin_LIBRARIES}
)
add_dependencies(follower_tracker ${catkin_EXPORTED_TARGETS})

add_executable(follower_tracker_node src/follower_tracker_node.cpp)
target_link_libraries(follower_tracker_node follower_tracker)

add_executable(global_path_plan_node src/global_path_plan_node.cpp)
target_link_libraries(global_path_plan_node ${catkin_LIBRARIES})

# unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_state_detect
                    test/state_detect.test
                    test/test_state_detect.cpp)
  target_link_libraries(test_state_detect ${catkin_LIBRARIES})
endif()

