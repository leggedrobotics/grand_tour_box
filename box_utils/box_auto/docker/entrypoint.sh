#!/bin/bash

figlet Welcome Docker - Box Auto


export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

# Enable sudo access without password
echo "root ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers
echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers

export HOSTNAME
export DISPLAY=:0.0
export PS1="\e[0;34m(box_auto) \e[0;32m\u@\h:\e[0;34m\W\$ \e[0m"

# Proceed as host user with superuser permissions
if [ $# -gt 0 ]; then
    sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc -c "source /opt/ros/noetic/setup.bash && source /home/rsl/catkin_ws/devel/setup.bash && source /etc/bash.bashrc && $@"
else
    sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc
fi

figlet Exit Docker - Box Auto