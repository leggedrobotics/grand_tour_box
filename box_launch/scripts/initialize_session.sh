#!/bin/bash

# Kill previous process
rosrun launch_scripts cleanup_session.sh

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "rosrun launch_scripts cleanup_session.sh"'