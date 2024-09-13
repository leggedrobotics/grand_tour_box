export HOSTNAME
export DISPLAY=:0.0
export ROS_MASTER_URI=http://192.168.2.57:11311
export ROS_IP=192.168.2.57

roslaunch bringup_fkie fkie.launch autostart_launch_files:="$(rospack find launch_pi)/launch/pi_recording.launch" use_gui:=false