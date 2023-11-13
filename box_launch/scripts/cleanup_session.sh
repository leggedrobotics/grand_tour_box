#!/bin/bash

# Kill all running screen sessions
killall -KILL screen
screen -wipe

# Kill node manager process
killall node_manager
killall roscore
killall rosmaster

# Clean PWM if on pi
if [ $(hostname) == "rpi" ]
then
    pigs p 19 0  # Set GPIO19 dutycycle to 0 (off)  [https://abyz.me.uk/rpi/pigpio/pigs.html#P/PWM]
    echo "Cleaned Up pigpiod"

pid=$(pgrep rosmaster)
if [ -z "$pid" ]
then
    echo "ROS was not running."
fi
else
    killall rosmaster
    kill -9 $pid
    tail --pid=$pid -f /dev/null # wait untill the process finishes / is fully killed
    echo "ROSmaster was stopped."
fi
echo "Cleaned Up Session"
