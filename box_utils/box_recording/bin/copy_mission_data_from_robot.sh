#! /bin/bash

if [ "$#" -ne 3 ]; then
  echo "Example usage: `basename $0` both ~/bags"
  exit 0
fi

user="rsl"
robot_name=$1
# Allow "~" to be used to specify home path: https://unix.stackexchange.com/a/399442
local_path_to_save_data=$3
local_path_to_save_data="${local_path_to_save_data//\~/$HOME}"

# today=$(date +"%Y_%m_%d")
# local_path_to_save_bags=${local_path_to_save_data}/${robot_name}_mission_${today}
# local_path_to_save_logs=${local_path_to_save_bags}/${robot_name}_logs

if [ ! -d "${local_path_to_save_bags}" ]; then
  mkdir -p "${local_path_to_save_bags}"
fi

if [ "$robot_name" == "jetson" ] || [ "$robot_name" == "Jetson" ] || [ "$robot_name" == "both" ]; then
  # Jetson: mission rosbags, background rosbags, tmux log and git log
  rsync --progress ${user}@jetson:~/git/grand_tour_box/box_utils/box_recording/data/* ${local_path_to_save_data}
  # rsync --progress ${user}@192.168.2.51:~/mission_log/* ${local_path_to_save_logs}
  
elif [ "$robot_name" == "nuc" ] || [ "$robot_name" == "Nuc" ] || [ "$robot_name" == "NUC" ] || [ "$robot_name" == "both" ]; then
  # NUC: mission rosbags, background rosbags, tmux log and git log
  rsync --progress ${user}@nuc:~/git/grand_tour_box/box_utils/box_recording/data/* ${local_path_to_save_data}
  # rsync --progress ${user}@192.168.2.56:~/mission_log/* ${local_path_to_save_logs}
fi
