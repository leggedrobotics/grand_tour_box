# Kill all running screen sessions
killall -KILL screen
screen -wipe

# Kill node manager process
killall node_manager
killall roscore
killall rosmaster

echo "Cleaned Up Session"