#!/bin/bash

#alias tk='tmux kill-server'
#alias tk_dev='tmux kill-session -a'

alias l-npc='tmuxp load /home/integration/boxi_scripts_ws/src/grand_tour_box/box_launch/tmux/npc.yaml'

alias sync-clocks="sudo killall chronyd; sudo chronyd"

alias tk="tmux kill-session -t npc-box"