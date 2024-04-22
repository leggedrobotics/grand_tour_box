cmake_minimum_required(VERSION 3.9)
project(ap20_driver_ros)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(ap20_driver_ros src/main.cpp)

target_link_libraries(ap20_driver_ros
  ${catkin_LIBRARIES}
)

install(TARGETS ap20_driver_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)