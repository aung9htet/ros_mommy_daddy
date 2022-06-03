cmake_minimum_required(VERSION 3.0.2)
project(ros_mommy_daddy_msg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Action.msg
  Emotion.msg
  RobotPub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy
  message_runtime
)