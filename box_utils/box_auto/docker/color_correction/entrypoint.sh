#!/bin/bash

source /opt/ros/noetic/setup.bash
echo $@
python3 /color_correction.py $@
