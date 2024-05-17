#!/bin/bash

# Kill previous process
/home/rsl/git/grand_tour_box/box_launch/scripts/cleanup_session.sh

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "/home/rsl/git/grand_tour_box/box_launch/scripts/cleanup_session.sh"'
