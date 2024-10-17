#!/bin/bash

# Manually on jetson netplan
sudo apt install netplan.io linuxptp tmux tmuxp

/home/rsl/git/grand_tour_box/box_configuration/general/general_install.sh

# Simlink only needed packages - we do this to not have ROS2 and ROS1 packages in our ROS1 workspace
mkdir -p ~/catkin_ws/src/grand_tour_box
ln -s ~/git/grand_tour_box/box_bringup ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_calibration ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_configuration ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_drivers ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_launch ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_model ~/catkin_ws/src/grand_tour_box/
ln -s ~/git/grand_tour_box/box_utils ~/catkin_ws/src/grand_tour_box/

sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan generate
sudo netplan apply

# with the network bridge ptp as a slave fails
#sudo cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/10-network-bridge.yaml /etc/netplan/10-network-bridge.yaml
#netplan generate
#sudo netplan apply

# (only one of the three things is actually doing the intended presistent modification no clue which one)
# modified here and commented in the line  net.ipv4.ip_forward=1
/etc/sysctl.conf
# Create file /etc/sysctl.d/99-forward.conf with not sure if this actually works
net.ipv4.ip_forward=1
# Allow to forward the internet connection to other PCs - this is lost after reboot
echo 1| sudo tee /proc/sys/net/ipv4/ip_forward


# Set IP tables temproarley
sudo iptables -A FORWARD -i mgbe0 -j ACCEPT
sudo iptables -A FORWARD -i mgbe1 -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -j ACCEPT

sudo iptables -t nat -A POSTROUTING -o mgbe0 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o mgbe1 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE


sudo apt install iptables-persistent -y
# here store it ip tables only for ipv4


# Install ssh server
sudo apt install openssh-server -y
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

# once copy over the .git folder for the fkie multi master
scp -r /home/rsl/git/grand_tour_box/box_drivers/multimaster_fkie jetson:/home/rsl/git/grand_tour_box/box_drivers/
cd ~/catkin_ws/src/grand_tour_box/box_drivers; rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie
cd ~/catkin_ws; catkin build fkie_multimaster

# Instal CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt -y install cuda

# Install Zed SDK
wget -O ~/Downloads/ZED_SDK_Tegra_L4T35.4_v4.1.1.zstd.run https://download.stereolabs.com/zedsdk/4.1/l4t35.4/jetsons
cd ~/Downloads
sudo apt install zstd -y
chmod +x ZED_SDK_Tegra_L4T35.4_v4.1.1.zstd.run
./ZED_SDK_Tegra_L4T35.4_v4.1.1.zstd.run


rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/jetson/.bashrc ~/

# .bashrc
# echo '' >> ~/.bashrc
# Prepend to .bashrc so that they still run when we are ssh'd into the PCs (when the interactive session is not started)
# echo 'source /opt/ros/noetic/setup.bash' | cat - ~/.bashrc > temp && mv temp ~/.bashrc
# echo 'source /home/rsl/catkin_ws/devel/setup.bash' | cat - ~/.bashrc > temp && mv temp ~/.bashrc
# echo 'source /home/rsl/git/grand_tour_box/box_configuration/jetson/alias.sh' | cat - ~/.bashrc > temp && mv temp ~/.bashrc

# autostart tmux
sudo cp ~/git/grand_tour_box/box_configuration/jetson/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
sudo systemctl daemon-reload
sudo systemctl enable autostart_tmux

# autostart ptp and set per default to jetson to grandmaster
# Maybe needed before CPT7 installation
#sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/ptp4l_mgbe1.service /lib/systemd/system/ptp4l_mgbe1.service
#sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/phc2sys_mgbe1.service /lib/systemd/system/phc2sys_mgbe1.service
#sudo systemctl disable ptp4l_mgbe1
#sudo systemctl disable phc2sys_mgbe1


sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/jetson_clocks_once.service /etc/systemd/system/jetson_clocks_once.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/ptp4l_mgbe0.service /lib/systemd/system/ptp4l_mgbe0.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/phc2sys_mgbe0.service /lib/systemd/system/phc2sys_mgbe0.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/box_chrony.service /lib/systemd/system/box_chrony.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/cleanup_ros_folder.service /etc/systemd/system/cleanup_ros_folder.service

sudo systemctl daemon-reload
sudo systemctl enable cleanup_ros_folder
sudo systemctl enable autostart_tmux

sudo systemctl enable ptp4l_mgbe0
sudo systemctl enable phc2sys_mgbe0
sudo systemctl enable box_chrony

sudo systemctl disable chrony
sync-clocks

# sync clocks script
sudo cp /home/rsl/git/grand_tour_box/box_configuration/jetson/sync_clocks_jetson.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/sync_clocks_jetson.sh

# Remove it then from boot
sudo apt install ntp
sudo service ntp stop
sudo update-rc.d -f ntp remove
sudo timedatectl set-ntp false
sudo systemctl stop ntp
sudo systemctl disable ntp


# max power mode
sudo nvpmodel -m 0
sudo /usr/bin/jetson_clocks

# deactivate bluetooth if the bluetooth settings are flickering (AX210)
# cp ~/catkin_ws/src/grand_tour_box/box_configuration/jetson/81-bluetooth-hci-rules /etc/udev/rules.d/81-bluetooth-hci.rules
# reboot -h now


# disable systemd-timesyncd and ntpd

# chrony
sudo apt install chrony -y
sudo vi /etc/chrony/chrony.conf
# comment out all the pool machines

# 2.2. How do I make an NTP server?
# By default, chronyd does not operate as an NTP server. You need to add an allow directive to the chrony.conf file in order for chronyd to open the server NTP port and respond to client requests.

# allow 192.168.1.0/24
# An allow directive with no specified subnet allows access from all IPv4 and IPv6 addresses.


chronyd -x
 

# CREATE UDEV RULES FOR HDR cameras
sudo vi /etc/udev/rules.d/99-video-devices.rules
#############################################
# Rule for /dev/video2 (tier4_isx021 41-001b)
SUBSYSTEM=="video4linux", ATTR{name}=="vi-output, tier4_isx021 41-001b", SYMLINK+="hdr_front"

# Rule for /dev/video3 (tier4_isx021 41-001c)
SUBSYSTEM=="video4linux", ATTR{name}=="vi-output, tier4_isx021 41-001c", SYMLINK+="hdr_right"

# Rule for /dev/video4 (tier4_isx021 41-001c)
SUBSYSTEM=="video4linux", ATTR{name}=="vi-output, tier4_isx021 42-001c", SYMLINK+="hdr_left"
#############################################