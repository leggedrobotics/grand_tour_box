#!/bin/bash

# Argument 1: Directory of the ros workspace.
# Argument 2: Directory containing the bags to be merged.
# Argument 3: Directory and name of the merged bag.

ROS_WS=""
if [ -z "$1" ]
then
			echo "Define ROS workspace as first argument: e.g. /home/user/catkin_ws/"
			exit 0
else
			ROS_WS=$1
fi
echo "ROS workspace:" $ROS_WS

INPUT_DIR=""
if [ -z "$2" ]
then
			INPUT_DIR=$PWD # if dir is empty take current directory.
else
			INPUT_DIR=$2
fi
echo "Input directory:" $INPUT_DIR

OUTPUT_DIR=""
OUTPUT_FILE=""
if [ -z "$3" ]
then
			OUTPUT_DIR=$INPUT_DIR
			# output file.
			SORTED_FILE_PATHS=$(ls -v $INPUT_DIR/*)
			for FILE_PATH in ${SORTED_FILE_PATHS}; do
				FILE_NAME=${FILE_PATH##*/}
				EXTENSION="${FILE_NAME#*.}"
				if [ "$EXTENSION" = "bag" ]
				then
					LPC_FILE=${FILE_NAME: -17}
					if [ "$LPC_FILE" = "lpc_mission_0.bag" ]
					then
						requested_number_of_characters=$((${#FILE_NAME}-${#LPC_FILE}))
						NEW_FILE_NAME="${FILE_NAME:0:$requested_number_of_characters}mission.bag"
						OUTPUT_FILE="${INPUT_DIR}/${NEW_FILE_NAME}" # if dir is empty take current directory.
						break
					else
						OUTPUT_FILE="${INPUT_DIR}/mission.bag"
					fi
				fi
			done
else
			OUTPUT_DIR=$(dirname $3)
			OUTPUT_FILE=$3
fi
echo "Output file:" $OUTPUT_FILE

MERGE_BAGS=""
# Create string of bags to be merged, it's important to order the chunks based on
# natural order with option "-v", so chunk _10 will be after chunk _9 and not after _1
SORTED_FILE_PATHS=$(ls -v $INPUT_DIR/*)
for FILE_PATH in ${SORTED_FILE_PATHS}; do
EXTENSION="${FILE_PATH#*.}"
	if [ "$EXTENSION" = "bag" ]
	then
		MERGE_BAGS="${MERGE_BAGS}${FILE_PATH} "
	fi
done

source $ROS_WS/devel/setup.bash
roscd anymal_rsl_recording/
python3 ./scripts/merge_bags.py -v $OUTPUT_FILE $MERGE_BAGS

# No compressing, because if depth images are compressed they can not be reconstructed.
# echo "Compressing merged bag $OUTPUT_FILE"
# rosbag compress --lz4 --output-dir=${OUTPUT_DIR} $OUTPUT_FILE

# # remove backup (uncompressed rosbag)
# backup_rosbag=${OUTPUT_FILE::-4}".orig.bag"
# echo "Removing uncompressed bag ${backup_rosbag}"
# rm ${backup_rosbag}
