boxi push --npc

# Ensure all hostnames are set correctly and you can ssh from every pc into every pc

sudo apt install ros-noetic-jsk-rviz-plugins -y
sudo apt install tmuxp -y
sudo apt install ros-${ROS_DISTRO}-novatel-oem7-driver -y

# SSH into npc
cd ~/catkin_ws/src
git clone https://github.com/fkie/multimaster_fkie.git
rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie


ln -s ~/git/grand_tour_box/box_utils/box_recording  ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_drivers/zed-ros-wrapper ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_drivers/zed2i_recording_driver ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_bringup/bringup_fkie ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_utils/box_health  ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_launch/box_launch  ~/catkin_ws/src/


catkin build bringup_fkie
catkin build box_recording
catkin build box_launch
catkin build box_health
# Append alias for autostart
roslaunch bringup_fkie fkie.launch autostart_launch_files:="$(rospack find box_health)/launch/box_health.launch $(rospack find box_recording)/launch/rosbag_record_node.launch" use_gui:=false

