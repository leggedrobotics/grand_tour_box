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
project(box_fan_control)

#==
# Dependencies
#==

find_package(catkin REQUIRED COMPONENTS
            message_generation
            std_msgs
)

add_service_files(
    FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


#==
# Package Configuration
#==

catkin_package(
    CATKIN_DEPENDS std_msgs
)

#==
# Package installation
#==

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# EOF
