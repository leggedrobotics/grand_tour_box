#=============================================================================

# Copyright (C) 2023, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
cmake_minimum_required(VERSION 3.10)
project(box_health)

#==
# Dependencies
#==

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)



add_message_files(
  FILES
  health_status_jetson.msg
  health_status_nuc.msg
  health_status_pi.msg
)

catkin_install_python(
  PROGRAMS
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#==
# Package Configuration
#==

catkin_package()

#==
# Package installation
#==


install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# EOF
