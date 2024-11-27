#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'
alias box='terminator -l box && exit'
alias robot='terminator -l box_robot && exit'
alias auto='terminator -l box_auto && exit'
alias reset_chrony='sudo systemctl restart chrony.service'

alias l-opc='tmuxp load $(rospack find box_launch)/../tmux/opc_recording.yaml'
alias l-calib-camera='tmuxp load $(rospack find box_launch)/../tmux/opc_calib_camera.yaml'
alias l-calib-lidar='tmuxp load $(rospack find box_launch)/../tmux/opc_calib_lidar.yaml'
alias l-calib-imu='tmuxp load $(rospack find box_launch)/../tmux/opc_calib_imu.yaml'
alias l-calib-prism='tmuxp load $(rospack find box_launch)/../tmux/opc_calib_prism.yaml'


alias l-replay='tmuxp load $(rospack find box_launch)/../tmux/opc_replay.yaml'
alias l-replay-zsh='tmuxp load $(rospack find box_launch)/../tmux/opc_replay_zsh.yaml'
alias stop_recording='rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"'
alias rosbag_play='~/git/grand_tour_box/box_launch/launch_replay/scripts/rosbag_play.sh'

alias ros_local='export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=127.0.0.1'


alias chrony_set_box="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo cp ~/git/grand_tour_box/box_configuration/opc/chrony.conf /etc/chrony/chrony.conf; sudo chronyd"
alias chrony_set_default="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo ln -s /etc/chrony/chrony_internet.conf /etc/chrony/chrony.conf; sudo chronyd"
alias sync-clocks="chrony_set_box; sudo killall chronyd; sudo chronyd"

alias terminator_pid='ps axo pid,wchan:32,cmd | grep terminator'