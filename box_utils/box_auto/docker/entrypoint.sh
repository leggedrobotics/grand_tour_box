#!/bin/bash

figlet Welcome GrandTour Docker


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
    source /opt/ros/noetic/setup.bash; source /home/opencv_ws/devel/setup.bash; source /home/catkin_ws/devel/setup.bash; $@
else
    bash --rcfile /root/.bashrc
fi


figlet Exit GrandTour Docker