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

# Define the custom PS1
custom_ps1="\[\e[0;34m\](box_auto) \[\e[0;32m\]\u@\h:\[\e[0;34m\]\W\$ \[\e[0m\]"

# Start interactive shell
bash --rcfile <(echo "PS1='$custom_ps1'")


figlet Exit GrandTour Docker