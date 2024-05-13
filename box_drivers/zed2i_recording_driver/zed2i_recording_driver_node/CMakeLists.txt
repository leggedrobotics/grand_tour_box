cmake_minimum_required(VERSION 3.5)
project(zed2i_recording_driver_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  zed2i_recording_driver_msgs
)

find_package(ZED 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

catkin_package(
  CATKIN_DEPENDS zed2i_recording_driver_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
