#!/bin/bash

#alias tk='tmux kill-server'
#alias tk_dev='tmux kill-session -a'

alias l-lpc='tmuxp load /home/${USER}/${GRANDTOUR_SCRIPT_WORKSPACE}/src/grand_tour_box/box_launch/tmux/lpc.yaml'

# Here sim-link does not work
alias chrony_set_box="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo cp /home/${USER}/${GRANDTOUR_SCRIPT_WORKSPACE}/src/grand_tour_box/box_configuration/anymal/lpc/chrony.conf /etc/chrony/chrony.conf; sudo chronyd"
alias chrony_set_default="sudo killall chronyd; sudo rm -f /etc/chrony/chrony.conf; sudo cp /etc/chrony/chrony.conf.orig /etc/chrony/chrony.conf; sudo chronyd"
alias sync-clocks="sudo killall chronyd; sudo chronyd"

alias tk="tmux kill-session -t lpc-box"