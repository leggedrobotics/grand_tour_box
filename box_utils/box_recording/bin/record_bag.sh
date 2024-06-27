#! /bin/bash

path=$1
topics="${@:2}"
bagfile="${path}".bag
#  --lz4
rosbag record --split --duration=5m -O "${bagfile}" $topics