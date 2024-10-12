#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-jetson='tmuxp load $(rospack find box_launch)/../tmux/jetson_recording.yaml'
# Launch mode that doesn't allow for recording
alias l-viz='tmuxp load $(rospack find box_launch)/../tmux/jetson_visualization.yaml'
alias l-calib='tmuxp load $(rospack find box_launch)/../tmux/jetson_calib.yaml'


alias attach-jetson='tmux attach-session -t jetson'
alias l-docker='/home/rsl/git/grand_tour_box/box_drivers_ros2/isaac_ros_common/scripts/run_recording.sh'
alias stop_recording='rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"'
alias sync-clocks='/home/rsl/git/grand_tour_box/box_configuration/jetson/sync_clocks_jetson.sh'