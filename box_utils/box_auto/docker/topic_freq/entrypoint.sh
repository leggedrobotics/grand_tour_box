#!/bin/bash

source /opt/ros/noetic/setup.bash
echo $@
python3 /topic_freq.py $@
