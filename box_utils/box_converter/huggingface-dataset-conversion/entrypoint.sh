#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.sh" --
source "/catkin_ws/devel/setup.sh" --

exec "$@"