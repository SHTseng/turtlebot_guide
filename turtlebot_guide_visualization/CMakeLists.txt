cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_guide_visualization)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  jsk_recognition_msgs
  turtlebot_guide_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS  
    roscpp
    geometry_msgs
    jsk_recognition_msgs
    turtlebot_guide_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(visualization_node src/visualization_node.cpp)
target_link_libraries(visualization_node ${catkin_LIBRARIES})
add_dependencies(visualization_node turtlebot_guide_msgs_generate_messages_cpp)
