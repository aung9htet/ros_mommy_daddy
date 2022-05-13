cmake_minimum_required(VERSION 3.0.2)
project(relationship_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Action.msg
  RobotPub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  rospy
  message_runtime
)

