cmake_minimum_required(VERSION 3.0.2)
project(core_throttling)

find_package(catkin REQUIRED)
catkin_package(
)

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)