#!/bin/bash

figlet Welcome GrandTour + Kleinkram Docker

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME
export KLEINKRAM_ACTIVE=FALSE

export HOSTNAME
export DISPLAY=:0.0

if [ $# -gt 0 ]; then
    source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
    eval "$@"
    exit $?
else
    bash --rcfile /root/.bashrc
fi

figlet Exit GrandTour Docker + KK