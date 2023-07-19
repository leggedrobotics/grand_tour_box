#!/bin/bash

# run config file
source ~/catkin_ws/src/grand_tour_box/box_utils/box_start/config.sh


# Wait for Jetson to be ready
echo "Waiting for Jetson"
until ping -c1 $JETSON_IP >/dev/null 2>&1; do :; done
echo "Found Jetson"

# Wait for the rosmaster on the Jetson to be ready
echo "Waiting for Rosmaster"
until rosnode ping -c1 "rosout" >/dev/null 2>&1; do :; done
echo "Found rosmaster"