#!/bin/bash

# run config file
source config.sh

# start Jetson
echo "Starting Jetson"
sshpass -p 'rsl' ssh -o StrictHostKeyChecking=no rsl@$JETSON_IP <<EOF
    cd ~/catkin_ws/src/grand_tour_box
    git pull

    # Run Startup
    ./box_utils/box_start/autostart_jetson.sh

    # Exit SSH session
    exit
EOF
echo "Jetson started"


# start NUC
echo "Starting Nuc"
sshpass -p 'rsl' ssh -o StrictHostKeyChecking=no rsl@$NUC_IP <<EOF
    cd ~/catkin_ws/src/grand_tour_box
    git pull

    # Run Startup
    ./box_utils/box_start/autostart_nuc.sh

    # Exit SSH session
    exit
EOF
echo "Nuc started"