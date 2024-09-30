#!/bin/bash

figlet Welcome GrandTour + Kleinkram Docker

klein endpoint set $ENDPOINT
klein login --key $APIKEY
mkdir "/mission_data"
mkdir "/mission_data/mission"
klein mission download $MISSION_UUID "/mission_data/mission"


export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

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
    source /home/opencv_ws/devel/setup.bash || { echo "Failed to source OpenCV"; exit 1; }
    source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }
    exec "$@"
else
    bash --rcfile /root/.bashrc
fi


figlet Exit GrandTour Docker