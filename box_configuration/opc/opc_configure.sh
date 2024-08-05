# For JSK visualization
wget http://archive.ubuntu.com/ubuntu/pool/universe/p/pcl/libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb
sudo dpkg -i libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb

cd ~/catkin_ws/src/grand_tour_box/box_drivers
rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie

rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/opc/.bashrc ~/


# roslaunch bringup_fkie fkie.launch autostart_launch_files:="$(rospack find box_recording)/launch/rosbag_record_node.launch" use_gui:=false