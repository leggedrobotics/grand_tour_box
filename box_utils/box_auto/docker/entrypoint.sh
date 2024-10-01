#!/bin/bash

figlet Welcome GrandTour Docker

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

export HOSTNAME
export DISPLAY=:0.0

if [ $# -gt 0 ]; then
    source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
    source /home/opencv_gtsam_ws/devel/setup.bash || { echo "Failed to source OpenCV"; exit 1; }
    source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }
    "$@"
else
    bash --rcfile /root/.bashrc
fi


figlet Exit GrandTour Docker