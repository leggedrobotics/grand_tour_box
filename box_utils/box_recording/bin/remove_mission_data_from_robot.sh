#! /bin/bash

# TODO: finalize this with pc of robot

if [ "$#" -ne 2 ]; then
  echo "Example usage: `basename $0` user cerberus"
  exit 0
fi

read -p "Are you REALLY REALLY REALLY sure you want to remove all rosbags and mission logs from on-board PCs? [y/n]" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
  user=$1
  robot_name=$2

  # NUC
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/.ros/anymal_lpc*'
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/mission_log/*'
  echo "Removed mission data from LPC"

  # Jetson
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/.ros/anymal_jetson*'
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/mission_log/*'
  echo "Removed mission data from Jetson"
fi
