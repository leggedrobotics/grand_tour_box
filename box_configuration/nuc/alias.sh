#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-nuc='tmuxp load $(rospack find box_launch)/../tmux/nuc_recording.yaml'
alias l-set-time='tmuxp load $(rospack find box_launch)/../tmux/nuc_set_time.yaml'
alias l-calib='tmuxp load $(rospack find box_launch)/../tmux/nuc_calib.yaml'

alias attach-nuc='tmux attach-session -t nuc'
alias stop_recording='rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"'
alias sync-clocks='/home/rsl/git/grand_tour_box/box_configuration/nuc/sync_clocks_nuc.sh'