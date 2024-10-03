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
project(launch_replay)

#==
# Dependencies
#==

find_package(catkin REQUIRED)

#==
# Package Configuration
#==

catkin_package()

#==
# Package Configuration
#==

catkin_package()

install(
  PROGRAMS  scripts/rosbag_play.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# EOF