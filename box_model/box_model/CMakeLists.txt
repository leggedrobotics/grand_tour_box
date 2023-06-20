cmake_minimum_required(VERSION 3.0.2)
project(box_model)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(CATKIN_DEPENDS xacro)

set(xacro_files
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/box/phasma.urdf.xacro

)
xacro_add_files(${xacro_files} INSTALL DESTINATION urdf)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

