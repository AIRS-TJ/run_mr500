cmake_minimum_required(VERSION 2.8.3)
project(ekf_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS 
std_msgs
nav_msgs
tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ekf_odom  src/ekf_odom.cpp)
target_link_libraries(ekf_odom  ${catkin_LIBRARIES})




