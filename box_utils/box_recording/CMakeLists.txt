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


# Find Protobuf
#find_package(Protobuf REQUIRED)
#find_package(gRPC REQUIRED)

# Set the directory for the generated Python files
#set(PROTO_GEN_PY_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen_py)

# Make sure the directory exists
#file(MAKE_DIRECTORY ${PROTO_GEN_PY_DIR})

# List of proto files
# set(PROTO_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/proto/recording_status.proto
#     ${CMAKE_CURRENT_SOURCE_DIR}/proto/start_recording.proto
# )

# Generate Python files from proto files  
# protobuf_generate_python(PROTO_PY_SRCS ${PROTO_GEN_PY_DIR} ${PROTO_FILES})
# This line before did not work - replaced with black magic
# Previous working without grpc dependency # 

# foreach(PROTO_FILE ${PROTO_FILES})
#     get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME_WE)
#     add_custom_command(
#         OUTPUT ${PROTO_GEN_PY_DIR}/${PROTO_FILE_NAME}_pb2.py
#         COMMAND ${Protobuf_PROTOC_EXECUTABLE}
#         ARGS --python_out=${PROTO_GEN_PY_DIR}  --grpc_python_out=${PROTO_GEN_PY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR}/proto ${PROTO_FILE}
#         DEPENDS ${PROTO_FILE}
#         COMMENT "Generating ${PROTO_GEN_PY_DIR}/${PROTO_FILE_NAME}_pb2.py"
#         VERBATIM
#     )
#     list(APPEND PROTO_PY_SRCS ${PROTO_GEN_PY_DIR}/${PROTO_FILE_NAME}_pb2.py)
# endforeach()


# Add a custom command to ensure the node.py script can find the generated protobuf modules
#add_custom_target(link_protobuf_files ALL
#    COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROTO_GEN_PY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/proto
#    DEPENDS ${PROTO_PY_SRCS}
#)

# Install the generated Python files
#install(DIRECTORY ${PROTO_GEN_PY_DIR}/
#        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/proto
#        FILES_MATCHING PATTERN "*.py"
#)


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
