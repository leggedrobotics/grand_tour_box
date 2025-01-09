# Exit if a single command fails
set -e

# Create workspace and configure
mkdir -p /home/catkin_ws/src
source /opt/ros/noetic/setup.bash
source /home/opencv_gtsam_ws/devel/setup.bash
cd /home/catkin_ws
catkin init
catkin config --extend /home/opencv_gtsam_ws/devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# grand_tour_box repository
cd /home/catkin_ws/src; git clone --recurse-submodules --shallow-submodules git@github.com:leggedrobotics/grand_tour_box.git --depth 1

# Dependencies: raw_image_pipeline
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/pybind11_catkin.git --depth 1
cd /home/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git --depth 1
cd /home/catkin_ws/src; git clone https://github.com/ethz-asl/glog_catkin.git --depth 1

# Dependencies: open3d_slam_private
cd /home/catkin_ws/src;  git clone https://github.com/leggedrobotics/message_logger.git --depth 1
# Dependencies (not sure if needed): graph_msf_dev
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/libnabo.git --depth 1


# Nice to have usability
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.vimrc ~/
mkdir -p ~/.vim/colors
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/solarized.vim ~/.vim/colors/
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.tmux.conf ~/


# Dependencies: open3d_slam_private - Upgrade cmake version to 3.19.2
apt update -y
apt install wget software-properties-common -y
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
apt update -y
apt install kitware-archive-keyring -y
apt install cmake=3.19.2-0kitware1ubuntu20.04.1 cmake-data=3.19.2-0kitware1ubuntu20.04.1 -y
apt install libgoogle-glog-dev libglfw3-dev liblua5.2-dev -y
apt install python3-catkin-tools libc++-dev libc++abi-dev -y 
apt install libxinerama-dev -y
apt install libxcursor-dev -y
apt install ros-noetic-tf-conversions -y
# Use: install_deps_ubuntu.sh [ assume-yes ]
set -ev

SUDO=${SUDO:=sudo} # SUDO=command in docker (running as root, sudo not available)
if [ "$1" == "assume-yes" ]; then
    APT_CONFIRM="--assume-yes"
else
    APT_CONFIRM=""
fi

dependencies=(
    # Open3D deps
    xorg-dev
    libglu1-mesa-dev
    python3-dev
    # Filament build-from-source deps
    libsdl2-dev
    libc++-7-dev
    libc++abi-7-dev
    ninja-build
    libxi-dev
    # OpenBLAS build-from-source deps
    gfortran
    # ML deps
    libtbb-dev
    # Headless rendering deps
    libosmesa6-dev
    # RealSense deps
    libudev-dev
    autoconf
    libtool
)
for package in "${dependencies[@]}"; do
    apt install "$APT_CONFIRM" "$package" -y
done


# Dependencies: novatel_oem7_driver - cpt7
cd /home/catkin_ws/src/grand_tour_box/box_drivers/novatel_oem7_driver
rosdep update
rosdep install --from-paths src --ignore-src -r -y --reinstall
cd /home/catkin_ws/src/grand_tour_box/box_applications/tf_bag
rosdep install --from-paths . --ignore-src -r -y --reinstall
sudo apt-get install ros-noetic-gps-common

# Dependencies: box_auto - hesai.py - dlio.py
pip3 install psutil

# Dependencies (not sure which package exactly): box_auto
pip3 install pandas
pip3 install scipy
pip3 install colorlog
pip3 install rosbags
pip3 install matplotlib # graph_msf
pip3 uninstall kleinkram -y
pip3 install kleinkram

# Dependencies for box_auto imu_timesync
pip3 install torch
pip3 install rerun-sdk

# 
pip3 install sortedcontainers
pip3 install rosnumpy

# Dependencies: box_auto - hesai.py
mkdir -p /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config
ln -s /home/catkin_ws/src/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32_firetime_correction.csv  /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32_firetime_correction.csv
ln -s /home/catkin_ws/src/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32.csv  /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32.csv

# Dependencies: box_auto - evo_evaluation
pip3 install fpdf -y

# Dependencies: direct_lidar_inertial_odometry
apt install -y libomp-dev libpcl-dev libeigen3-dev
apt install -y ros-noetic-pcl-ros

# Dependencies: tf_bag 
pip3 install importlib-metadata==4.13.0

# Dependencies: graph_msf_dev 
apt install -y libeigen3-dev
apt install -y ros-noetic-kdl-parser
apt install -y ros-noetic-eigen-conversions
apt install -y ros-noetic-image-transport
apt install -y ros-noetic-compressed-depth-image-transport
apt install -y ros-noetic-diagnostic-updater
apt install -y libgflags-dev
apt install -y ros-noetic-interactive-markers
apt install -y ros-noetic-tf2-eigen


apt update -y

# Build workspace
source /opt/ros/noetic/setup.bash
source /home/opencv_gtsam_ws/devel/setup.bash
cd /home/catkin_ws

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}

catkin build direct_lidar_inertial_odometry
catkin build box_auto

echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Install boxi
cd /home/catkin_ws/src/grand_tour_box/box_utils/boxi; pip3 install -e ./ --no-cache-dir
