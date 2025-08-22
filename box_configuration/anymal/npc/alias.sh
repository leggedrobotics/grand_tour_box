#!/bin/bash

#alias tk='tmux kill-server'
#alias tk_dev='tmux kill-session -a'

# This is the alais to run on the NPC of the robot.
alias l-npc='tmuxp load /home/${USER}/${GRANDTOUR_SCRIPT_WORKSPACE}/src/grand_tour_box/box_launch/tmux/npc.yaml'

alias sync-clocks="sudo killall chronyd; sudo chronyd"

alias tk="tmux kill-session -t npc-box"