cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guide_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  base_local_planner
  tf
  tf_conversions
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    nav_core
    base_local_planner
    tf
    tf_conversions
    geometry_msgs
    sensor_msgs
    nav_msgs
    move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(global_path_plan_node src/global_path_plan_node.cpp)
target_link_libraries(global_path_plan_node ${catkin_LIBRARIES})

add_executable(turtlebot_guide_node src/turtlebot_guide_node.cpp)
target_link_libraries(turtlebot_guide_node ${catkin_LIBRARIES})

add_executable(map_odom_publisher_node src/map_odom_publisher.cpp)
target_link_libraries(map_odom_publisher_node ${catkin_LIBRARIES})

# unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_spawn_robot
                    test/spawn_robot.test
                    test/test_spawn_robot.cpp)
  target_link_libraries(test_spawn_robot ${catkin_LIBRARIES})

  add_rostest_gtest(test_single_ir_camera
                    test/single_ir_camera.test
                    test/test_single_ir_camera.cpp)
  target_link_libraries(test_single_ir_camera ${catkin_LIBRARIES})

  add_rostest_gtest(test_multi_ir_camera
                    test/multi_ir_camera.test
                    test/test_multi_ir_camera.cpp)
  target_link_libraries(test_multi_ir_camera ${catkin_LIBRARIES})
endif()

