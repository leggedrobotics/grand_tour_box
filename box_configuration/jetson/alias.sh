#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-autonomy='tmuxp load $(rospack find box_launch)/../tmux/jetson_autonomy.yaml'
alias l-recording='tmuxp load $(rospack find box_launch)/../tmux/jetson_recording.yaml'
# Launch mode that doesn't allow for recording
alias l-viz='tmuxp load $(rospack find box_launch)/../tmux/jetson_visualization.yaml'
alias l-calib-camera='tmuxp load $(rospack find box_launch)/../tmux/jetson_calib_camera.yaml'
alias l-calib-lidar='tmuxp load $(rospack find box_launch)/../tmux/jetson_calib_lidar.yaml'
alias l-calib-imu='tmuxp load $(rospack find box_launch)/../tmux/jetson_calib_imu.yaml'
alias l-calib-prism='tmuxp load $(rospack find box_launch)/../tmux/jetson_calib_prism.yaml'


alias attach-jetson='tmux attach-session -t jetson'
alias l-docker='/data/workspaces/ros2_ws/src/isaac_ros_common/scripts/run_recording.sh'
alias stop_recording='rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"'
alias sync-clocks='/home/rsl/git/grand_tour_box/box_configuration/jetson/sync_clocks_jetson.sh'