# Exit if a single command fails
set -e

# Install ros
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update

apt install -y ros-noetic-ros-base python3-catkin-tools
# apt install -y ros-noetic-desktop-full python3-catkin-tools 

source /opt/ros/noetic/setup.bash
apt install -y python3-rosbag
apt install -y ros-noetic-cv-bridge
apt install -y python3-catkin-tools
apt install -y python3-rosdep

echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

rosdep init
rosdep update