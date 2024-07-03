# For JSK visualization
wget http://archive.ubuntu.com/ubuntu/pool/universe/p/pcl/libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb
sudo dpkg -i libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb



# roslaunch bringup_fkie fkie.launch autostart_launch_files:="$(rospack find box_recording)/launch/rosbag_record_node.launch" use_gui:=false