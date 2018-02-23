cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guide_integration)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  gazebo_ros
  geometry_msgs
)

find_package(TinyXML2 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    geometry_msgs
  DEPENDS
    TinyXML2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(guide_straight_path src/guide_straight_path.cpp)
target_link_libraries(guide_straight_path ${catkin_LIBRARIES})

add_executable(actor_waypoints_generator src/actor_waypoints_generator.cpp)
target_link_libraries(actor_waypoints_generator ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
