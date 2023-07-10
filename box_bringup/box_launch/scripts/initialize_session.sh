#!/bin/bash

# Kill previous process
rosrun box_launch cleanup_session.sh

set -g mode-mouse on
set -g mouse-select-pane on
set -g mouse-resize-pane on
set -g mouse-select-window on

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "rosrun box_launch cleanup_session.sh"'