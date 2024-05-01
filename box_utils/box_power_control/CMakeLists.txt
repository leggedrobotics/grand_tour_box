#=============================================================================
# Copyright (C) 2024, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
cmake_minimum_required(VERSION 3.10)
project(box_power_control)

#==
# Dependencies
#==

find_package(catkin REQUIRED COMPONENTS
            std_msgs
            std_srvs
)

#==
# Package Configuration
#==

catkin_package(
    CATKIN_DEPENDS std_msgs std_srvs message_runtime
)

#==
# Package installation
#==

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# EOF
