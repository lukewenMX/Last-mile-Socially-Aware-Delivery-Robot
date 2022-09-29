cmake_minimum_required(VERSION 2.8.3)
project(sidewalk_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  # Needed to generate custom message types
  genmsg
  std_msgs
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
)

add_message_files(  # Tell catkin which custom message files to include
  FILES
  Boundary.msg
  Trajectory.msg
  Trajectories.msg
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs # Needed to generate custom messages
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)

