#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/catkin_ws/devel/setup.bash" --

echo $KLEINKRAM_CONFIG > ~/.kleinkram.json

exec "$@"
