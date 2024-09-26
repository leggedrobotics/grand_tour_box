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
add_executable(demo_cpp_node 
  src/demo_node.cpp
)

target_link_libraries(demo_cpp_node
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS demo_cpp_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



