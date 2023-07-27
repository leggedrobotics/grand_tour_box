#! /bin/bash

if [ "$#" -ne 2 ]; then
  echo "Example usage: `basename $0` rsl jetson"
  exit 0
fi

read -p "Are you REALLY REALLY REALLY sure you want to remove all rosbags and mission logs from on-board PCs? [y/n]" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
  user=$1
  robot_name=$2

  # Jetson
  if [ "$robot_name" == "jetson" ] || [ "$robot_name" == "Jetson" ]; then
    #ssh ${user}@192.168.2.51 -t 'rm ~/.ros/anymal_lpc*'
    ssh ${user}@192.168.2.51 -t 'rm ~/git/grand_tour_box/box_utils/box_recording/data/*'
    #ssh ${user}@192.168.2.51 -t 'rm ~/mission_log/*'
    echo "Removed mission data from Jetson"
    
  elif [ "$robot_name" == "nuc" ] || [ "$robot_name" == "Nuc" ] || [ "$robot_name" == "NUC" ]; then
    # NUC
    #ssh ${user}@192.168.2.56 -t 'rm ~/.ros/anymal_jetson*'
    ssh ${user}@192.168.2.56 -t 'rm ~/git/grand_tour_box/box_utils/box_recording/data/*'
    #ssh ${user}@192.168.2.56 -t 'rm ~/mission_log/*'
    echo "Removed mission data from NUC"
  fi
fi
