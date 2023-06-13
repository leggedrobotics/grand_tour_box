#!/bin/bash
# Prepare tmux session
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Kill previous process
roslaunch box_launch cleanup_session.sh

# run cleanup script on exit
tmux set-hook -g 'session-closed' 'run "roslaunch box_launch cleanup_session.sh"'