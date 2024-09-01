
# Exit if a single command fails
set -e


# Cloning repositories
mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src; git clone --recurse-submodules git@github.com:leggedrobotics/grand_tour_box.git
cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/pybind11_catkin.git
cd /home/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git
cd /home/catkin_ws/src; git clone https://github.com/ethz-asl/glog_catkin.git

# Nice to have usability
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.vimrc ~/
mkdir -p ~/.vim/colors
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/solarized.vim ~/.vim/colors/
cp /home/catkin_ws/src/grand_tour_box/box_configuration/general/.tmux.conf ~/

# Setting up workspace
source /opt/ros/noetic/setup.sh; 
cd /home/catkin_ws; 
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Building workspace
catkin build box_auto

# Special installations missing potentially
# cd /home/rsl/git/grand_tour_box/box_drivers/novatel_oem7_driver
# rosdep install --from-paths src --ignore-src -r -y --reinstall
# ./build.sh -f