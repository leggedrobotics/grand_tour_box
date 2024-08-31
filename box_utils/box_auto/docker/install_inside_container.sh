sudo apt-get install -y vim tmux tmuxp 

# Install ros
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update
apt install -y ros-noetic-desktop-full python3-catkin-tools 


# Cloning repositories
mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src; git clone --recurse-submodules git@github.com:leggedrobotics/grand_tour_box.git
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/raw_image_pipeline.git
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/pybind11_catkin.git
cd /home/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git
cd /home/catkin_ws/src; git clone https://github.com/ethz-asl/glog_catkin.git


# Nice to have usability
cp /home/catkin_ws/grand_tour_box/box_configuration/general/.vimrc ~/
mkdir -p ~/.vim/colors
cp /home/catkin_ws/grand_tour_box/box_configuration/general/solarized.vim ~/.vim/colors/
cp /home/catkin_ws/grand_tour_box/box_configuration/general/.tmux.conf ~/

# Installing dependencies
apt-get update && apt install -y python3-catkin-tools git libyaml-cpp-dev libtool

# Setting up workspace
source /opt/ros/noetic/setup.sh; 
cd /home/catkin_ws; 
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Building workspace
catkin build raw_image_pipeline_ros
catkin build raw_image_pipeline_python

# Special installations missing potentially
# cd /home/rsl/git/grand_tour_box/box_drivers/novatel_oem7_driver
# rosdep install --from-paths src --ignore-src -r -y --reinstall
# ./build.sh -f


# Install python packages
apt install -y python3-rosbag
apt install -y ros-noetic-cv-bridge
apt install -y python3-opencv
apt install -y python3-pip
pip3 install tqdm 
pip3 install pytictac