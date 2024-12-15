useradd -m -s /bin/bash -G sudo rsl
echo 'rsl:rsl' | chpasswd
echo 'rsl ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers


# Setup GrandTour Repo
apt update
mkdir -p /home/rsl/git 
mkdir -p /home/rsl/ros2_ws/src
cd /home/rsl/git; git clone git@github.com:leggedrobotics/grand_tour_box.git

# Install MCAP conversion tool
cd /bin; wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.47/mcap-linux-amd64
chmod +x /bin/mcap-linux-amd64
cp /bin/mcap-linux-amd64 mcap


# OKVIS Installation
cd /home/rsl/git; git clone --recurse-submodules git@github.com:smartroboticslab/okvis2.git
ln -s ~/git/okvis2 ~/ros2_ws/src
apt install -y cmake
apt install -y  cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libboost-dev libboost-filesystem-dev apt install libopencv-dev

# Install realsense SDK -> maybe this is not needed in a minimal version
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt install librealsense2-dev -y

# CV Bridge
sudo apt install ros-humble-cv-bridge

# Build ROS2 workspace
colcon build --cmake-args -DUSE_NN=OFF

