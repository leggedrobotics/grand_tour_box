#!/bin/bash

# run config file
source config.sh

# source ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
sleep 3

# Wait for Jetson to be ready
until ping -c1 $JETSON_IP >/dev/null 2>&1; do :; done
echo "Found Jetson"

tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml