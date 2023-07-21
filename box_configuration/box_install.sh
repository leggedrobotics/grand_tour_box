#!/bin/bash

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full python3-catkin-tools

# Make catkin workspace
mkdir -p ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Clone grand_tour_box repo
mkdir -p ~/git
cd ~/git
git clone --recurse-submodules git@github.com:leggedrobotics/grand_tour_box.git
cd ~/git/grand_tour_box
ln -s ~/git/grand_tour_box ~/catkin_ws/src
cd ~/catkin_ws

# Install driver dependencies
sudo apt install libv4l-dev v4l-utils
sudo apt install libpcap-dev

cd ~/catkin_ws/src/grand_tour_box/box_drivers/Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install

sudo apt install python3-pip
pip3 install pyserial

sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install -i --as-root pip:false --reinstall --from-paths ~/catkin_ws/src/grand_tour_box/box_drivers/multimaster_fkie
cd ~/catkin_ws/src/grand_tour_box/box_drivers
rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie
catkin build fkie_multimaster

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
./grand_tour_box/box_drivers/ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh
git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:ethz-asl/libsbp_catkin.git
git clone git@github.com:ethz-asl/libserialport_catkin.git

cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/cuckoo_time_translator.git

curl -Ls http://deb.7sr.ch/pubkey.gpg | sudo gpg --dearmor -o /usr/share/keyrings/deb-7sr-ch-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] http://deb.7sr.ch/alphasense/stable $(lsb_release -cs) main" \
          | sudo tee /etc/apt/sources.list.d/sevensense.list
sudo apt update
sudo apt install alphasense-driver-core alphasense-viewer alphasense-firmware ros-noetic-alphasense-driver-ros ros-noetic-alphasense-driver
sudo sysctl -w net.core.rmem_max=11145728

sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino

sudo copy box_drivers/adis16475_ros/udev/70-adis16475.rules /etc/udev/rules.d/
sudo service udev reload  
sudo service udev restart

catkin build versavis_adis16448_receiver
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/grand_tour_box/box_drivers/smb_imu_interface/firmware

# Miscellaneous
sudo apt install netplan.io
sudo apt install linuxptp