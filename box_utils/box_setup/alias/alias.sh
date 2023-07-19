#!/bin/bash

alias tk='tmux kill-server'
alias gs='git status'
alias recho='rostopic echo'
alias rlist='rostopic list'
alias rhz='rostopic hz'
alias rinfo='rostopic info'

alias l-nuc='tmuxp load $(rospack find box_launch)/tmux/box_nuc.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/tmux/box_jetson.yaml'
alias ssh-nuc='ssh rsl@192.168.2.56'
alias ssh-jetson='ssh rsl@192.168.2.51'
alias attach-nuc='tmux attach-session -t box_nuc'
alias attach-jetson='tmux attach-session -t box_jetson'
alias restart-tmux='sudo systemctl restart autostart_tmux'

