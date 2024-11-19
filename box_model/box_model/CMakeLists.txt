cmake_minimum_required(VERSION 3.0.2)
project(box_model)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(CATKIN_DEPENDS xacro)

set(xacro_files
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/box/box.urdf.xacro

)
xacro_add_files(${xacro_files} INSTALL DESTINATION urdf)

catkin_install_python(
  PROGRAMS
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY config launch urdf scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

