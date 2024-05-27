#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-nuc='tmuxp load $(rospack find box_launch)/../tmux/nuc_recording.yaml'
alias l-pi='tmuxp load /home/rsl/git/grand_tour_box/box_launch/tmux/pi_recording.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/../tmux/jetson_recording.yaml'
alias l-opc='tmuxp load $(rospack find box_launch)/../tmux/opc_recording.yaml'
alias l-replay='tmuxp load $(rospack find box_launch)/../tmux/opc_replay.yaml'
alias attach-nuc='tmux attach-session -t nuc'
alias attach-jetson='tmux attach-session -t jetson'
alias attach-pi='tmux attach-session -t pi'
alias l-docker-pi='/home/rsl/git/grand_tour_box/box_configuration/pi/docker/run.sh'
alias l-docker-jetson='/data/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/run_dev.sh'
alias silence=' sudo pigpiod && pigs p 18 90'

start-recording(){
   rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: ''" 
}

start-recording-calibration(){
    rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: '/home/rsl/git/grand_tour_box/box_utils/box_recording/cfg/box_calibration.yaml'" 
}

start-recording-uncompressed(){
   rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: '/home/beni/git/grand_tour_box/box_utils/box_recording/cfg/box_uncompressed.yaml'" 
}

stop-recording(){
    rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false" 
}

export PATH=${HOME}/.local/bin${PATH:+:${PATH}}