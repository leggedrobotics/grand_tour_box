#!/bin/bash

JETSON_IP="192.168.1.51"
NUC_IP="192.168.1.56"


# start roscore
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
sleep 3

# Wait for NUC to be ready
until ping -c1 $JETSON_IP >/dev/null 2>&1; do :; done
echo "Found Jetson"

# Start USB drivers
tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml