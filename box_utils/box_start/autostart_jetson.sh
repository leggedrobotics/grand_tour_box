#!/bin/bash

# run config file
source config.sh

# start roscore
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roscore&
sleep 3

tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml