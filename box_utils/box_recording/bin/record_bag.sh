#! /bin/bash

path=$1
topics="${@:2}"
bagfile="${path}".bag
rosbag record --split --lz4 --duration=5m -O "${bagfile}" $topics