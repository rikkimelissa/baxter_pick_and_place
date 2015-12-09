cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  message_generation
  tf
  baxter_interface
  urdf_parser
  pykdl_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

