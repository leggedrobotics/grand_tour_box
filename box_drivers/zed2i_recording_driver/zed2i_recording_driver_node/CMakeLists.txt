cmake_minimum_required(VERSION 3.5)
project(zed2i_recording_driver_node)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  zed2i_recording_driver_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2_ros
)
find_package(Threads REQUIRED)

find_package(ZED 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS zed2i_recording_driver_msgs roscpp sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  Threads::Threads
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(recording_contracts tests/recording_contracts.cpp)
  if(TARGET recording_contracts)
    target_link_libraries(recording_contracts ${catkin_LIBRARIES} ${ZED_LIBRARIES} Threads::Threads)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
