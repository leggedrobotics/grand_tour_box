cmake_minimum_required(VERSION 3.0.2)
project(start_recording_svo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  message_generation
  std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES start_recording_svo.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs)