source /opt/ros/noetic/setup.bash
source /home/rsl/catkin_ws/devel/setup.bash
sleep 5

export HOSTNAME
export DISPLAY=:0.0
export ROS_MASTER_URI=http://192.168.2.57:11311
export ROS_IP=192.168.2.57
roslaunch bringup_fkie fkie_pi.launch \
    autostart_launch_files:="$(rospack find launch_pi)/launch/pi_recording.launch" \
    load_launch_files:="$(rospack find bringup_stim320)/launch/stim320.launch $(rospack find bringup_adis16475)/launch/adis16475.launch" \
    use_gui:=false