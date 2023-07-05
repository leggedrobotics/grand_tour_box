#! /bin/bash

path=$1
topics="${@:2}"
bagfile="${path}".bag
rosbag record --split --duration=5m --lz4 -O "${bagfile}" $topics