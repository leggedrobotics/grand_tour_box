#!/usr/bin/env bash

INPUT_PATH="/media/tutuna/T7/2024-08-14-10-45-39/"
mkdir -p ${INPUT_PATH}/combined_files
mkdir -p ${INPUT_PATH}/log

# Merge the separate files into 1 file. (i.e hesai_0 hesai_1 -> hesai)

# Might need to automaize this part
CPT_BAG=${INPUT_PATH}/2024-08-14-10-45-39_nuc_cpt7_0.bag
TF_BAG=${INPUT_PATH}/2024-08-14-10-45-39_lpc_tf_0.bag
HESAI_BAG=${INPUT_PATH}/2024-08-14-10-45-39_nuc_hesai_0.bag
ZED_SVO=${INPUT_PATH}/2024-08-14-10-45-39_jetson_zed2i.svo2
HDR_BAG=

cd ${INPUT_PATH}

clear

echo "Initiate Combined Replaying: ${INPUT_PATH}."

source /opt/ros/noetic/setup.bash
source /home/tutuna/box_ws/devel/setup.bash

sleep 2s

roslaunch novatel_oem7_driver cpt7_replay.launch input_bag_path:=${CPT_BAG}

sleep 2s

roslaunch box_post_processor box_post_processor.launch input_filepath:=${TF_BAG}

sleep 2s

roslaunch hesai_ros_driver replay_packets.launch input_rosbag_path:=${HESAI_BAG}

sleep 2s

roslaunch zed_wrapper zed2i_replay.launch svo_file:=${ZED_SVO}

sleep 2s

# Move the files to a new folder

# Select the important files to move

# Combine the files 

RED='\033[0;31m'
GREEN='\033[0;92m'
NC='\033[0m' # No Color
echo -e "${GREEN}Successfully generated the filesin ${INPUT_PATH} ${NC}"

# echo "Successfully generated resources in ${INPUT_PATH}."
exit 0