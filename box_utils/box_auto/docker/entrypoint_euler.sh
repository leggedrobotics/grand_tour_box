#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash || { echo "Failed to source ROS"; exit 1; }
source /home/opencv_gtsam_ws/devel/setup.bash || { echo "Failed to source OpenCV"; exit 1; }
source /home/catkin_ws/devel/setup.bash || { echo "Failed to source Catkin workspace"; exit 1; }

echo $KLEINKRAM_CONFIG > ~/.kleinkram.json

exec "$@"
