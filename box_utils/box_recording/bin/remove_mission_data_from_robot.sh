#! /bin/bash

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

  # LPC
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/.ros/anymal_lpc*'
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-lpc -t 'rm ~/mission_log/*'
  echo "Removed mission data from LPC"

  # NPC
  ssh ${user}@anymal-${robot_name}-npc -t 'rm ~/.ros/anymal_npc*'
  ssh ${user}@anymal-${robot_name}-npc -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-npc -t 'rm ~/mission_log/*'
  echo "Removed mission data from NPC"

  # Jetson
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/.ros/anymal_jetson*'
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-jetson -t 'rm ~/mission_log/*'
  echo "Removed mission data from Jetson"

  # APC
  ssh ${user}@anymal-${robot_name}-apc -t 'rm ~/.ros/anymal_apc*'
  ssh ${user}@anymal-${robot_name}-apc -t 'rm ~/git/anymal_rsl/anymal_rsl/anymal_rsl_utils/anymal_rsl_recording/anymal_rsl_recording/data/*'
  ssh ${user}@anymal-${robot_name}-apc -t 'rm ~/mission_log/*'
  echo "Removed mission data from APC"

fi
