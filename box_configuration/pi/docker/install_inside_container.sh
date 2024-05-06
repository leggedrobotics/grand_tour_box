
sudo apt-get update
sudo apt-get install -y build-essential
sudo apt install -y python-setuptools python3-setuptools
cd /home/rsl/git/pigpio-master
make
sudo make install
# Verify working
./x_pigpio

# Installing fkie multi master
cd /home/rsl/git/grand_tour_box/box_drivers
sudo apt-get install -y python3-rosdep
sudo rosdep init
rosdep update


sudo apt-get install vim -y
sudo apt-get install ros-noetic-catkin python3-catkin-tools -y


mkdir -p /home/rsl/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd /home/rsl/catkin_ws
ln -s ~/git/grand_tour_box ~/catkin_ws/src/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo



catkin build fkie_multimaster
catkin build launch_pi

sudo apt-get update
sudo apt install python3-pip -y
sudo apt install python3-grpc-tools -y
pip3 install ruamel.yaml==0.15.89
pip3 install psutil
sudo apt install screen -y
sudo apt-get install figlet -y
# Install adis
cd /home/rsl/git/grand_tour_box/box_drivers/adis16475_driver/adis16475_driver && mkdir build && cd build
cmake .. && make
sudo make install


# Also ensure to add the correct source script to /etc/bash.bashrc