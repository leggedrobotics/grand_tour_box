#! /bin/bash

set -eu
path=$1
shift
# Keep the writer at the PID owned by the recorder service. The Python
# `rosbag record` launcher can exit on SIGINT before its child closes the bag.
# rosrun execs the C++ writer, so wait() now also waits for the bag index flush.
exec rosrun rosbag record --split --lz4 --duration=5m -O "${path}.bag" "$@"
