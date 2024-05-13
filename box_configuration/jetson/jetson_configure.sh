#!/bin/bash

# .bashrc
echo '' >> ~/.bashrc
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source catkin_ws/devel/setup.bash' >> ~/.bashrc
echo 'source catkin_ws/src/grand_tour_box/box_configuration/alias.sh' >> ~/.bashrc


# autostart roscore
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/env.sh /etc/ros/env.sh
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/roscore.service /etc/systemd/system/roscore.service
sudo systemctl roscore
sudo systemctl enable roscore

# autostart tmux
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
sudo systemctl daemon-reload
sudo systemctl enable autostart_tmux

# autostart ptp
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/sync_time_once.service /etc/systemd/system/sync_time_once.service
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/jetson_clocks_once.service /etc/systemd/system/jetson_clocks_once.service
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/ptp4l_mgbe0.service /lib/systemd/system/ptp4l_mgbe0.service
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/ptp4l_mgbe1.service /lib/systemd/system/ptp4l_mgbe1.service
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/phc2sys_mgbe0.service /lib/systemd/system/phc2sys_mgbe0.service
sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/phc2sys_mgbe1.service /lib/systemd/system/phc2sys_mgbe1.service
sudo systemctl daemon-reload
sudo systemctl enable sync_time_once
sudo systemctl enable jetson_clocks_once
sudo systemctl enable ptp4l_mgbe0
sudo systemctl enable ptp4l_mgbe1
sudo systemctl enable phc2sys_mgbe0
sudo systemctl enable phc2sys_mgbe1



# netplan
cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
netplan generate
netplan apply

# deactivate bluetooth if the bluetooth settings are flickering (AX210)
# cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/81-bluetooth-hci-rules /etc/udev/rules.d/81-bluetooth-hci.rules
# reboot -h now