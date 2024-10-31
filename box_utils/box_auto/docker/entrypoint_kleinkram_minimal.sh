#!/bin/bash

figlet Welcome GrandTour + Kleinkram Docker

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

echo  $APIKEY

klein endpoint set $ENDPOINT
klein login --key $APIKEY
mkdir -p "/mission_data"
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
    "$@"
else
    bash --rcfile /root/.bashrc
fi

figlet Exit GrandTour Docker + KK