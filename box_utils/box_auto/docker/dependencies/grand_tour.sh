# Exit if a single command fails
set -e

######################  Open3D-SLAM dependencies ######################
# Upgrade cmake version to 3.19.2
apt install wget software-properties-common -y
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
apt update -y
apt install kitware-archive-keyring -y
apt install cmake=3.19.2-0kitware1ubuntu20.04.1 cmake-data=3.19.2-0kitware1ubuntu20.04.1 -y
apt install libgoogle-glog-dev libglfw3-dev liblua5.2-dev -y
apt install python3-catkin-tools libc++-dev libc++abi-dev -y 


# Create workspace and configure
mkdir -p /home/catkin_ws/src
source /opt/ros/noetic/setup.bash
source /home/opencv_gtsam_ws/devel/setup.bash
cd /home/catkin_ws
catkin init
catkin config --extend /home/opencv_gtsam_ws/devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Get the grand_tour_box repository and its submodules
cd /home/catkin_ws/src; git clone -b feat/kleinkramIntegration --single-branch --recurse-submodules git@github.com:leggedrobotics/grand_tour_box.git

# Dependency for raw_image_pipeline
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/pybind11_catkin.git
cd /home/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git
cd /home/catkin_ws/src; git clone https://github.com/ethz-asl/glog_catkin.git

# Dependency for open3d_slam_private
cd /home/catkin_ws/src;  git clone git@github.com:leggedrobotics/message_logger.git
# Maybe only needed for graph_msf_dev
cd /home/catkin_ws/src; git clone git@github.com:leggedrobotics/libnabo.git


# Added simlink to access the hesai files correctly
mkdir -p /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config
ln -s /home/catkin_ws/src/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32_firetime_correction.csv  /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32_firetime_correction.csv
ln -s /home/catkin_ws/src/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32.csv  /home/rsl/git/grand_tour_box/box_bringup/bringup_hesai/config/PandarXT-32.csv

# Nice to have usability
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.vimrc ~/
mkdir -p ~/.vim/colors
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/solarized.vim ~/.vim/colors/
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.tmux.conf ~/

# Install cpt7 dependencies
cd /home/catkin_ws/src/grand_tour_box/box_drivers/novatel_oem7_driver
rosdep update
rosdep install --from-paths src --ignore-src -r -y --reinstall

cd /home/catkin_ws/src/grand_tour_box/box_drivers/tf_bag
rosdep install --from-paths . --ignore-src -r -y --reinstall

# Build workspace for postprocessing
cd /home/catkin_ws
catkin build box_auto
echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

cd /home/catkin_ws/src/grand_tour_box/box_utils/boxi; pip3 install -e ./ --no-cache-dir