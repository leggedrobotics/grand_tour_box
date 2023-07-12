#!/bin/bash

# run config file
source catkin_ws/src/grand_tour_box/box_utils/box_start/config.sh

# start roscore
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export HOSTNAME

roscore&
sleep 1

#tmuxp load $(rospack find box_launch)/tmux/box_jetson.yaml
roslaunch $(rospack find box_launch)/launch/jetson_sensors.launch