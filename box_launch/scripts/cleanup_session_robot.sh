#!/bin/bash

# Kill node manager process
killall node_manager
yes y | rosnode cleanup
rm -rf /dev/shm/COSMO_SHM

if ps aux | grep '[f]kie' > /dev/null; then
  ps aux | grep '[f]kie' | awk '{print $2}' | xargs kill
fi