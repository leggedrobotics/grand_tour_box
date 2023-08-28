#!/bin/bash

# Kill all running screen sessions
killall -KILL screen
screen -wipe

# Kill node manager process
killall node_manager
killall roscore
killall rosmaster

killall -s SIGKILL rosmaster

pid=$(pgrep rosmaster)
if [ -z "$pid" ]
then
    echo "ROS was not running."
else
    killall rosmaster
    kill -9 $pid
    tail --pid=$pid -f /dev/null # wait untill the process finishes / is fully killed
    echo "ROSmaster was stopped."
fi
echo "Cleaned Up Session"