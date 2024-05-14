#!/bin/bash

# Manually on jetson netplan
sudo apt install netplan.io linuxptp tmux tmuxp

/home/rsl/git/grand_tour_box/box_configuration/general/general_install.sh

# add hosts
sudo echo "192.168.2.51	jetson" >> /etc/hosts
sudo echo "192.168.2.56	nuc" >> /etc/hosts
sudo echo "192.168.2.57	pi" >> /etc/hosts
sudo echo "192.168.2.154 opc" >> /etc/hosts

sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan generate
sudo netplan apply

sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/10-network-bridge.yaml /etc/netplan/10-network-bridge.yaml
netplan generate

# Magic commands to allow to forward the internet connection to other PCs
iptables -A FORWARD -i mgbe0 -j ACCEPT
iptables -A FORWARD -i mgbe1 -j ACCEPT
iptables -A FORWARD -i wlan0 -j ACCEPT

iptables -t nat -A POSTROUTING -o mgbe0 -j MASQUERADE
iptables -t nat -A POSTROUTING -o mgbe1 -j MASQUERADE
iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE

echo 1| sudo tee /proc/sys/net/ipv4/ip_forward

# Install ssh server
sudo apt-get install openssh-server -y
sudo ssh-keygen -A
sudo systemctl restart systemd-network


# Setup ssh keys copy over:
ssh rsl@192.168.2.51 -o PubkeyAuthentication=no -o PreferredAuthentications=password
# exit again and copy over the keys
scp -r -o PubkeyAuthentication=no -o PreferredAuthentications=password /home/rsl/.ssh/* rsl@192.168.2.51:/home/rsl/.ssh
# ssh into jetson and restart
ssh-add /home/rsl/.ssh/id_rsa
killall ssh-agent; eval "$(ssh-agent)"

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update -y
sudo apt install ros-noetic-desktop-full python3-catkin-tools -y

# Make catkin workspace
mkdir -p ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ln -s /home/rsl/git/grand_tour_box ~/catkin_ws/src

# Install fkie multimaster
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
sudo apt install python3-grpc-tools -y
cd ~/catkin_ws/src/grand_tour_box/box_drivers; rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie
cd ~/catkin_ws; catkin build fkie_multimaster


# .bashrc
echo '' >> ~/.bashrc
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source /home/rsl/catkin_ws/devel/setup.bash' >> ~/.bashrc
echo 'source /home/rsl/git/grand_tour_box/box_configuration/alias.sh' >> ~/.bashrc

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

# max power mode
sudo nvpmodel -m 0
sudo /usr/bin/jetson_clocks

# deactivate bluetooth if the bluetooth settings are flickering (AX210)
# cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/81-bluetooth-hci-rules /etc/udev/rules.d/81-bluetooth-hci.rules
# reboot -h now