#!/bin/bash

figlet Welcome GrandTour + Kleinkram Docker

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

klein endpoint server $KLEINKRAM_API_ENDPOINT $KLEINKRAM_S3_ENDPOINT
klein login --key $APIKEY

export KLEINKRAM_ACTIVE=ACTIVE

# Enable sudo access without password
# echo "root ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers
# echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers

export HOSTNAME
export DISPLAY=:0.0
# Start interactive shell
# Proceed as host user with superuser permissions
# Maybe add sourcing of .bashrc



if [ $# -gt 0 ]; then
    source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
    source /home/opencv_gtsam_ws/devel/setup.bash || { echo "Failed to source OpenCV"; exit 1; }
    source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }
    eval "$@"
    exit $?
else
    bash --rcfile /root/.bashrc
fi

figlet Exit GrandTour Docker + KK