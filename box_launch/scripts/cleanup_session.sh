#!/bin/bash

# Kill all running screen sessions
killall -KILL screen
screen -wipe

# Kill node manager process
killall node_manager
killall roscore
killall rosmaster

# Clean PWM if on pi
# if [ $(hostname) == "pi" ]
# then
#    pigs p 19 0  # Set GPIO19 dutycycle to 0 (off)  [https://abyz.me.uk/rpi/pigpio/pigs.html#P/PWM]
#    echo "Cleaned Up pigpiod"

pid=$(pgrep rosmaster)
if [ -z "$pid" ]
then
   echo "ROS was not running."
else
echo $pid
    killall rosmaster
    kill -9 $pid
    tail --pid=$pid -f /dev/null # wait untill the process finishes / is fully killed
    echo "ROSmaster was stopped."
fi

if [ $(hostname) == "jetson" ]
then
   ~git/grand_tour_box/box_drivers/ap20_driver/ap20_driver_ros/script/ap20_stop_rosrover.sh
fi

echo "Cleaned Up Session"

if ps aux | grep '[f]kie' > /dev/null; then
  ps aux | grep '[f]kie' | awk '{print $2}' | xargs kill
fi