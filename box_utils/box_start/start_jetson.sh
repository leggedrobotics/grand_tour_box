#!/bin/bash

JETSON_IP="192.168.1.51"
NUC_IP="192.168.1.56"

#echo $JETSON_IP


# start roscore
source /opt/ros/noetic/setup.bash
roscore

echo $JETSON_IP
rostopic list