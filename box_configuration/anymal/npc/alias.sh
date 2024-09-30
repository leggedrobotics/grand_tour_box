#!/bin/bash

#alias tk='tmux kill-server'
#alias tk_dev='tmux kill-session -a'

alias l-npc='tmuxp load /home/rsl/git/grand_tour_box/box_launch/tmux/npc.yaml'

# Here sim-link does not work
alias chrony_set_box="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo cp /home/rsl/git/grand_tour_box/box_configuration/anymal/npc/chrony.conf /etc/chrony/chrony.conf; sudo chronyd"
alias chrony_set_default="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo cp /etc/chrony/grand_tour_box/chrony_robot.conf /etc/chrony/chrony.conf; sudo chronyd"
alias sync-clocks="chrony_set_box; sudo killall chronyd; sudo chronyd"

alias tk="tmux kill-session -t npc-box"