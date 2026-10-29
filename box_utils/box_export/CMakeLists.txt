cmake_minimum_required(VERSION 3.5)
project(box_export)

# ------------------------------------------------------------------------------
# Build settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-O3)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  eigen_conversions
  nav_msgs
  geometry_msgs
  rosgraph_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag
  interactive_markers
  sensor_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------
# catkin package
# ------------------------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS OpenCV Eigen3
)

# ------------------------------------------------------------------------------
# Includes
# ------------------------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Targets
# NOTE: Do NOT build a library from files that contain `main()`.
# ------------------------------------------------------------------------------
add_executable(zed2i_decompression_and_saving
  src/zed2i_decompress_and_format_data.cpp
)
add_dependencies(zed2i_decompression_and_saving ${catkin_EXPORTED_TARGETS})
target_link_libraries(zed2i_decompression_and_saving
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  zed2i_decompression_and_saving
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
