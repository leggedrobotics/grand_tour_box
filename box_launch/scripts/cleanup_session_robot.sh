#!/bin/bash

# Kill node manager process
killall node_manager
yes y | rosnode cleanup
rm -rf /dev/shm/COSMO_SHM

# On the robot
export GRANDTOUR_WORKSPACE="i_hate_docker_ws"
export GRANDTOUR_SCRIPT_WORKSPACE="boxi_scripts_ws"

# On the Box
export BOX_WORKSPACE="catkin_ws"

if ps aux | grep '[f]kie' > /dev/null; then
  ps aux | grep '[f]kie' | awk '{print $2}' | xargs kill
fi