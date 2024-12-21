#!/bin/bash

figlet GrandTour Docker ROS2

if [ $# -gt 0 ]; then
    source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
    source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }
    "$@"
else
    bash --rcfile /root/.bashrc
fi

figlet Exit GrandTour Docker + KK