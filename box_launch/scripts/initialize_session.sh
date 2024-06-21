#!/bin/bash

# Kill previous process
~/git/grand_tour_box/box_launch/scripts/cleanup_session.sh

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "~/git/grand_tour_box/box_launch/scripts/cleanup_session.sh"'