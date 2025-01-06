cmake_minimum_required(VERSION 3.5.1)
project(colorless_uniform_mapper)

set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)
set(CMAKE_COMPILER_IS_GNUCXX 1)

set(CATKIN_PACKAGE_DEPENDENCIES
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  tf2_msgs
  tf2_ros
  std_srvs
  visualization_msgs
)

# PCL 1.10 is standard on Ubuntu 20.04
find_package(PCL 1.10 EXACT REQUIRED PATHS /usr/include/pcl-1.10/)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
  set(Magenta     "${Esc}[35m")
endif()

message("${Magenta}CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}.${ColourReset}")
message("${Magenta}CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}.${ColourReset}")
message("${Magenta}CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}.${ColourReset}")
message("${Magenta}CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}.${ColourReset}")
message("${Magenta}CMAKE_FIND_ROOT_PATH: ${CMAKE_FIND_ROOT_PATH}.${ColourReset}")
message("${Magenta}CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}.${ColourReset}")

###########
## Build ##
###########
# Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ColorlessMapper.cpp
  src/${PROJECT_NAME}/circularRingBuffer.cpp
)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
  )
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Rosnode executable
add_executable(${PROJECT_NAME}_node
  src/colorless_uniform_mapper_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
)
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##############
##   gtest Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      roslaunch
  )
  roslaunch_add_file_check(launch)

  ## Currently only an empty test exists.
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${catkin_LIBRARIES}
  )
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)