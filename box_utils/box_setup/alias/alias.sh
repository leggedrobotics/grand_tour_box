#!/bin/bash

alias tk='tmux kill-server'
alias gs='git status'
alias recho='rostopic echo'
alias rlist='rostopic list'
alias rhz='rostopic hz'
alias rinfo='rostopic info'

alias l-nuc='tmuxp load $(rospack find box_launch)/tmux/box_nuc.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/tmux/box_jetson.yaml'
alias l-opc='tmuxp load $(rospack find box_launch)/tmux/box_opc.yaml'
alias ssh-nuc='sshpass -p rsl ssh -X rsl@192.168.2.56'
alias ssh-jetson='sshpass -p rsl ssh -X rsl@192.168.2.51'
alias ping-nuc='ping 192.168.2.56'
alias ping-jetson='ping 192.168.2.51'
alias attach-nuc='tmux attach-session -t box_nuc'
alias attach-jetson='tmux attach-session -t box_jetson'
alias restart-tmux='sudo systemctl restart autostart_tmux'

alias reboot-nuc='sshpass -p rsl ssh -t rsl@192.168.2.56 "echo rsl | sudo -S reboot"'
alias reboot-jetson='sshpass -p rsl ssh -t rsl@192.168.2.51 "echo rsl | sudo -S reboot"'
alias reboot-box='reboot-nuc; reboot-jetson'

alias shutdown-nuc='sshpass -p rsl ssh -t rsl@192.168.2.56 "echo rsl | sudo -S shutdown -h now"'
alias shutdown-jetson='sshpass -p rsl ssh -t rsl@192.168.2.51 "echo rsl | sudo -S shutdown -h now"'
alias shutdown-box='shutdown-nuc; shutdown-jetson'