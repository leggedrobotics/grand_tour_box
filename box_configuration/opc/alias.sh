#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'
alias box='terminator -l box && exit'
alias reset_chrony='sudo systemctl restart chrony.service'

alias l-opc='tmuxp load $(rospack find box_launch)/../tmux/opc_recording.yaml'
alias l-replay='tmuxp load $(rospack find box_launch)/../tmux/opc_replay.yaml'