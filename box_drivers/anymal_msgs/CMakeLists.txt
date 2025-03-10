cmake_minimum_required(VERSION 3.0.2)
project(anymal_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  SeActuatorReadings.msg
  SeActuatorReading.msg
  SeActuatorState.msg
  SeActuatorCommand.msg
  BatteryState.msg
  AnymalState.msg
  Command.msg
  Contact.msg
  ExtendedJointState.msg
  Mode.msg
  ReadingExtended.msg
  ReadingsExtended.msg
  StampedFloat32MultiArray.msg
  StateExtended.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
                 message_generation
                 message_runtime
                 std_msgs
                 sensor_msgs
)
