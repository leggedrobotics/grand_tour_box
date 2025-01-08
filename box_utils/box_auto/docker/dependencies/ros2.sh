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


# OKVIS Installation is currently not working
cd /home/rsl/git; git clone --recurse-submodules git@github.com:smartroboticslab/okvis2.git
ln -s ~/git/okvis2 ~/ros2_ws/src
apt install -y cmake
apt install -y  cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libboost-dev libboost-filesystem-dev libopencv-dev

# # Install realsense SDK -> maybe this is not needed in a minimal version
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
apt-get install apt-transport-https -y
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list

apt-get update -y
apt install librealsense2-dev -y

# CV Bridge
apt install ros-humble-cv-bridge -y

# For Running OKVIZ example
apt install xvfb -y

# # Build ROS2 workspace
cd /home/rsl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DUSE_NN=OFF

# Example commnad
# ~/ros2_ws/build/okvis$ xvfb-run ./okvis_app_synchronous /home/rsl/ros2_ws/src/okvis2/config/euroc.yaml /tmp_disk/euroc/MH_01_easy/mav0/