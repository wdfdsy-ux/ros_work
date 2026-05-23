cmake_minimum_required(VERSION 3.0.2)
project(remove_back_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_pose
  move_base_msgs
  relative_move
  roscpp
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES remove_back_pkg
#  CATKIN_DEPENDS actionlib ar_pose move_base_msgs relative_move roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(re_back_node src/re_back.cpp)
target_link_libraries(re_back_node ${catkin_LIBRARIES})
