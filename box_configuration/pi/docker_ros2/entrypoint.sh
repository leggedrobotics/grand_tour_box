#!/bin/bash

figlet Welcome Docker ROS2

export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

# Enable sudo access without password
echo "root ALL=(ALL) NOPASSWD:ALL" >> sudo v
echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> sudo /etc/sudoers

# Proceed as host user with superuser permissions
if [ $# -gt 0 ]; then
    sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc -c "source /opt/ros/humble/setup.bash && source /etc/bash.bashrc && $@"
    # h && source /home/rsl/catkin_ws/devel/setup.bash 
else
    sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc
fi

figlet Exit Docker ROS2