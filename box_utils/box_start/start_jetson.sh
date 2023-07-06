#!/bin/bash

JETSON_IP="192.168.1.51"
NUC_IP="192.168.1.56"

#echo $JETSON_IP


# start roscore
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore&
sleep 3


# Wait for NUC to be ready
until ping -c1 $NUC_IP >/dev/null 2>&1; do :; done
echo "Found NUC"

# Start fkie multimaster
tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml