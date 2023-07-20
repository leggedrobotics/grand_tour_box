#!/bin/bash

# .bashrc
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source catkin_ws/devel/setup.bash' >> ~/.bashrc
echo 'source catkin_ws/src/grand_tour_box/box_utils/box_setup/alias/alias.sh' >> ~/.bashrc
echo '\n' >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://192.168.2.51:11311' >> ~/.bashrc
echo 'export ROS_IP=192.168.2.51' >> ~/.bashrc


# autostart tmux
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
systemctl daemon-reload
systemctl enable autostart_tmux

# autostart ptp
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/ptp4l_mgbe0.service /lib/systemd/system/ptp4l_mgbe0.service
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/ptp4l_mgbe1.service /lib/systemd/system/ptp4l_mgbe1.service
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/phc2sys_mgbe0.service /lib/systemd/system/phc2sys_mgbe0.service
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/phc2sys_mgbe1.service /lib/systemd/system/phc2sys_mgbe1.service
systemctl daemon-reload
systemctl enable ptp4l_mgbe0
systemctl enable ptp4l_mgbe1
systemctl enable phc2sys_mgbe0
systemctl enable phc2sys_mgbe1

# netplan
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
netplan generate
netplan apply

# deactivate bluetooth if the bluetooth settings are flickering (AX210)
# cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/81-bluetooth-hci-rules /etc/udev/rules.d/81-bluetooth-hci.rules
# reboot -h now