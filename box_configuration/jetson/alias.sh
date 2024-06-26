#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-jetson='tmuxp load $(rospack find box_launch)/../tmux/jetson_recording.yaml'
# Launch mode that doesn't allow for recording
alias l-jetson-viz='tmuxp load $(rospack find box_launch)/../tmux/jetson_visualization.yaml'

alias attach-jetson='tmux attach-session -t jetson'
alias l-docker-jetson='/data/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/run_recording.sh'

alias sync-clocks='/home/rsl/git/grand_tour_box/box_configuration/jetson/sync_clocks_jetson.sh'