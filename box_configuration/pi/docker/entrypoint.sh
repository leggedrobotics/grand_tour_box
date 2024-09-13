#!/bin/bash

figlet Welcome Docker

source ~/.bashrc

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

# Enable sudo access without password
# echo "root ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers
# echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers

export HOSTNAME
export DISPLAY=:0.0
export ROS_MASTER_URI=http://192.168.2.57:11311
export ROS_IP=192.168.2.57

# TO make this work we additionally manually modified the /etc/bash.bashrc file
export PS1="\e[0;34m(docker) \e[0;32m\u@\h:\e[0;34m\W\$ \e[0m"

# Proceed as host user with superuser permissions
# /home/rsl/.bashrc


if [ $# -gt 0 ]; then
    sudo bash --rcfile /etc/bash.bashrc -c "source ~/.bashrc && source /opt/ros/noetic/setup.bash && source /home/rsl/catkin_ws/devel/setup.bash && source /etc/bash.bashrc && $@"
else
    sudo bash --rcfile /etc/bash.bashrc
fi

figlet Exit Docker