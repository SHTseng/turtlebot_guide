cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guide_integration)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(guide_straight_path src/guide_straight_path.cpp)
target_link_libraries(guide_straight_path ${catkin_LIBRARIES})
