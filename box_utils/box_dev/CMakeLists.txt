cmake_minimum_required(VERSION 3.0.2)
project(box_dev)

find_package(catkin REQUIRED COMPONENTS
  rospy
  grid_map_msgs
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Filter.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    grid_map_msgs
    std_msgs
    dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  scripts/update_elevation_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
) 