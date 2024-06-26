#!/bin/bash

alias tk='tmux kill-server'
alias tk_dev='tmux kill-session -a'

alias l-nuc='tmuxp load $(rospack find box_launch)/../tmux/nuc_recording.yaml'

alias attach-nuc='tmux attach-session -t nuc'

alias sync-clocks='sync_clocks_nuc.sh'