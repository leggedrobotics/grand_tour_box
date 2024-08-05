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
project(box_recording)

#==
# Dependencies
#==

find_package(catkin REQUIRED COMPONENTS
            message_generation
            std_msgs
            zed2i_recording_driver_msgs
)


add_service_files(
    DIRECTORY srv
    FILES
    StartRecording.srv
    StartRecordingInternal.srv
    StopRecording.srv
    StopRecordingInternal.srv
)
catkin_python_setup()

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

install(DIRECTORY bin data scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# EOF
