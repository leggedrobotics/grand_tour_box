cmake_minimum_required(VERSION 3.0.2)
project(boxi_rectify_from_bag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  box_export
  roscpp
  rosbag
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 4 REQUIRED)

option(BOXI_USE_TURBOJPEG "Use libjpeg-turbo when available" ON)

set(LIBJPEG_TURBO_ROOT "/opt/libjpeg-turbo" CACHE PATH
  "Preferred root of the libjpeg-turbo installation")

set(BOXI_HAS_TURBOJPEG FALSE)
if(BOXI_USE_TURBOJPEG)
  find_path(TURBOJPEG_INCLUDE_DIR
    NAMES turbojpeg.h
    HINTS
      "${LIBJPEG_TURBO_ROOT}/include"
    NO_DEFAULT_PATH
  )

  find_library(TURBOJPEG_LIBRARY
    NAMES turbojpeg
    HINTS
      "${LIBJPEG_TURBO_ROOT}/lib64"
      "${LIBJPEG_TURBO_ROOT}/lib"
    NO_DEFAULT_PATH
  )

  if(NOT TURBOJPEG_INCLUDE_DIR OR NOT TURBOJPEG_LIBRARY)
    find_path(TURBOJPEG_INCLUDE_DIR NAMES turbojpeg.h)
    find_library(TURBOJPEG_LIBRARY NAMES turbojpeg)
  endif()

  if(TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
    set(BOXI_HAS_TURBOJPEG TRUE)
    get_filename_component(TURBOJPEG_LIBRARY_DIR "${TURBOJPEG_LIBRARY}" DIRECTORY)
    message(STATUS "Using TurboJPEG include dir: ${TURBOJPEG_INCLUDE_DIR}")
    message(STATUS "Using TurboJPEG library: ${TURBOJPEG_LIBRARY}")
  else()
    message(WARNING
      "TurboJPEG was not found. Falling back to OpenCV JPEG decode/encode. "
      "Install libjpeg-turbo-official or provide LIBJPEG_TURBO_ROOT to enable TurboJPEG.")
  endif()
else()
  message(STATUS "TurboJPEG support disabled. Using OpenCV JPEG decode/encode.")
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if(BOXI_HAS_TURBOJPEG)
  add_library(boxi_turbojpeg INTERFACE)
  target_include_directories(boxi_turbojpeg INTERFACE ${TURBOJPEG_INCLUDE_DIR})
  target_link_libraries(boxi_turbojpeg INTERFACE ${TURBOJPEG_LIBRARY})
endif()

function(configure_boxi_target target_name)
  target_link_libraries(${target_name}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  if(BOXI_HAS_TURBOJPEG)
    target_link_libraries(${target_name} boxi_turbojpeg)
    target_compile_definitions(${target_name} PRIVATE BOXI_HAS_TURBOJPEG=1)
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${TURBOJPEG_LIBRARY_DIR}"
      INSTALL_RPATH "${TURBOJPEG_LIBRARY_DIR}"
      LINK_FLAGS "-Wl,--disable-new-dtags"
    )
  else()
    target_compile_definitions(${target_name} PRIVATE BOXI_HAS_TURBOJPEG=0)
  endif()
endfunction()

add_executable(rectify_from_bag_node src/rectify_from_bag.cpp)
configure_boxi_target(rectify_from_bag_node)

add_executable(visualize_rectified_from_bag_node src/visualize_rectified_from_bag.cpp)
configure_boxi_target(visualize_rectified_from_bag_node)

add_executable(rectify_and_write_bag_node src/rectify_and_write_bag_node.cpp)
configure_boxi_target(rectify_and_write_bag_node)
add_dependencies(rectify_and_write_bag_node ${catkin_EXPORTED_TARGETS})

install(TARGETS rectify_from_bag_node visualize_rectified_from_bag_node rectify_and_write_bag_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
