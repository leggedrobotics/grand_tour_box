#! /bin/bash

# TODO: finalize this with pc of robot

if [ "$#" -ne 3 ]; then
  echo "Example usage: `basename $0` rsl jetson ~/bags"
  exit 0
fi

user=$1
robot_name=$2
# Allow "~" to be used to specify home path: https://unix.stackexchange.com/a/399442
local_path_to_save_data=$3
local_path_to_save_data="${local_path_to_save_data//\~/$HOME}"

today=$(date +"%Y_%m_%d")
local_path_to_save_bags=${local_path_to_save_data}/${robot_name}_mission_${today}
local_path_to_save_logs=${local_path_to_save_bags}/${robot_name}_logs

if [ ! -d "${local_path_to_save_bags}" ]; then
  mkdir -p "${local_path_to_save_bags}"
fi

if [ ! -d "${local_path_to_save_logs}" ]; then
  mkdir -p "${local_path_to_save_logs}"
fi

# NUC: mission rosbags, background rosbags, tmux log and git log
rsync --progress ${user}@anymal-${robot_name}-lpc:~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/* ${local_path_to_save_bags}
rsync --progress ${user}@anymal-${robot_name}-lpc:~/mission_log/* ${local_path_to_save_logs}

# Jetson: mission rosbags, background rosbags, tmux log and git log
rsync --progress ${user}@anymal-${robot_name}-jetson:~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/* ${local_path_to_save_bags}
rsync --progress ${user}@anymal-${robot_name}-jetson:~/mission_log/* ${local_path_to_save_logs}
