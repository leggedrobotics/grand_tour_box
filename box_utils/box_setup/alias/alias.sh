#!/bin/bash

alias tk='tmux kill-server'
alias gs='git status'
alias recho='rostopic echo'
alias rlist='rostopic list'
alias rhz='rostopic hz'
alias rinfo='rostopic info'

alias l-nuc='tmuxp load $(rospack find box_launch)/tmux/box_nuc.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/tmux/box_jetson.yaml'


