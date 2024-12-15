#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-recording='tmuxp load /home/rsl/git/grand_tour_box/box_launch/tmux/pi_recording.yaml'

alias attach-pi='tmux attach-session -t pi'
alias l-docker='/home/rsl/git/grand_tour_box/box_configuration/pi/docker/run.sh'
alias l-docker-ros2='/home/rsl/git/grand_tour_box/box_configuration/pi/docker_ros2/run.sh'
alias silence='sudo pigpiod; sleep 1; pigs p 18 180'