cmake_minimum_required(VERSION 3.5)
project(box_auto)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Source: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
#add_compile_options(-Wall -Wextra -Wpedantic -O3)

set(CMAKE_BUILD_TYPE "Release")
#add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-dev -O3 -std=c++17 -fconcepts )
set(CMAKE_COMPILER_IS_GNUCXX 1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
  src/box_imu_processor.cpp
  src/box_tf_processor.cpp
  src/merge.cpp
)

find_package(PCL 1.10 EXACT REQUIRED PATHS /usr/include/pcl-1.10/)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

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
  rospy
  cv_bridge
  visualization_msgs
  open3d_catkin
  anymal_msgs
  gnss_msgs
  any_velodyne_pointcloud
  box_model
  )

# We pass the path of the `FindXYZ.cmake` files to `CMAKE_MODULE_PATH`
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

# You can individually give the path of the libraries as well.
#list(APPEND LIBRARIES ".....so.0")
#list(APPEND LIBRARIES ".......so")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-Dbox_imu_processor_OPENMP_FOUND=${OpenMP_FOUND})
endif ()
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  )


catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  yaml-cpp
  ${PROJECT_NAME}
  CATKIN_DEPENDS 
  ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

## Link libraries
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


#Include directory#
include_directories(${PROJECT_SOURCE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

add_definitions(-std=c++17)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
  )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${OpenMP_CXX_LIBRARIES}
  )

add_executable(post_processor_node
  src/post_processor_node.cpp)

add_executable(tf_processor_node
  src/tf_processor_node.cpp)
  
add_executable(filter_lidar_node
  src/filter_lidar.cpp)

add_executable(merge
  src/merge.cpp)


add_executable(zed2i_decompression_and_saving src/zed2i_decompress_and_format_data.cpp)
add_executable(evo_conversion_node src/evo_evaluation_converter.cpp)
  
target_link_libraries(post_processor_node 
${catkin_LIBRARIES}
${gflags_LIBRARIES}
${GLOG_LIBRARIES}
${PROJECT_NAME}
)

target_link_libraries(tf_processor_node 
${catkin_LIBRARIES}
${gflags_LIBRARIES}
${GLOG_LIBRARIES}
${PROJECT_NAME}
)

target_link_libraries(merge 
${catkin_LIBRARIES}
${gflags_LIBRARIES}
${GLOG_LIBRARIES}
${PROJECT_NAME}
)

target_link_libraries(filter_lidar_node 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_NAME}
)

target_link_libraries(zed2i_decompression_and_saving ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(evo_conversion_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})


find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " post_processor_node filter_lidar_node tf_processor_node zed2i_decompression_and_saving evo_conversion_node merge)
  add_clang_tooling(
    TARGETS post_processor_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
    #CT_ATTACH_TO_ALL # Uncomment to attach to all targets during build, takes time
  )
endif(cmake_clang_tools_FOUND)


install(TARGETS post_processor_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS tf_processor_node
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS filter_lidar_node
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS zed2i_decompression_and_saving
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS evo_conversion_node
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS merge
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
