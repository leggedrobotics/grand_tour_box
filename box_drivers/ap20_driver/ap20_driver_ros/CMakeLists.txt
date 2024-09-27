cmake_minimum_required(VERSION 3.9)
project(ap20_driver_ros)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    rosconsole
)

# Add message files
add_message_files(
  FILES
  DebugInfo.msg
  ImuDebug.msg
  PositionDebug.msg
  TimestampDebug.msg
  TimestampKappiDebug.msg
)


# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_install_python(
  PROGRAMS
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime 
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# add_executable(ap20_driver_ros 
#   src/main.cpp
# )


add_executable(ap20_driver_dev 
  src/demo_node.cpp
)

add_executable(ap20_driver_node 
  src/simple_ap20_node.cpp
)


target_link_libraries(ap20_driver_node
  ${catkin_LIBRARIES}
)

target_link_libraries(ap20_driver_dev
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ap20_driver_dev
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ap20_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



