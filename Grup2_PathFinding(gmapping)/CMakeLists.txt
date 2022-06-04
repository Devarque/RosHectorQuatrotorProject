cmake_minimum_required(VERSION 3.0.2)
project(robot_tasarim)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  hector_uav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_tasarim
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


