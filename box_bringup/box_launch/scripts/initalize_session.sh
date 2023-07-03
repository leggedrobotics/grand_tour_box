#!/bin/bash

# Kill previous process
rosrun box_launch cleanup_session.sh

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "rosrun box_launch cleanup_session.sh"'