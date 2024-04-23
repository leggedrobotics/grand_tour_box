#!/bin/bash

alias tk='tmux kill-server'

alias l-nuc='tmuxp load $(rospack find box_launch)/../tmux/nuc_recording.yaml'
alias l-pi='tmuxp load $(rospack find box_launch)/../tmux/pi_recording.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/../tmux/jetson_recording.yaml'
alias l-opc='tmuxp load $(rospack find box_launch)/../tmux/opc_recording.yaml'
alias l-replay='tmuxp load $(rospack find box_launch)/../tmux/opc_replay.yaml'
alias attach-nuc='tmux attach-session -t nuc'
alias attach-jetson='tmux attach-session -t jetson'
alias attach-pi='tmux attach-session -t pi'


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