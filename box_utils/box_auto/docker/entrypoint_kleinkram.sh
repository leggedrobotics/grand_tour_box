#!/bin/bash

figlet Welcome GrandTour + Kleinkram Docker

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

klein endpoint set $ENDPOINT
klein login --key $APIKEY
mkdir "/mission_data"
klein mission download $MISSION_UUID "/mission_data"

# Enable sudo access without password
# echo "root ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers
# echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers

export HOSTNAME
export DISPLAY=:0.0
# Start interactive shell
# Proceed as host user with superuser permissions
# Maybe add sourcing of .bashrc

export KLEINKRAM_ACTIVE=true

if [ $# -gt 0 ]; then
    source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
    source /home/opencv_gtsam_ws/devel/setup.bash || { echo "Failed to source OpenCV"; exit 1; }
    source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }
    "$@"
else
    bash --rcfile /root/.bashrc
fi

klein mission tag $MISSION_UUID "8525480c-aa6c-4c8c-9229-8193f7c8de77" "true" 
## UUID of tagtype "processed". Thinking about allowing the tagtype name.

figlet Exit GrandTour Docker + KK