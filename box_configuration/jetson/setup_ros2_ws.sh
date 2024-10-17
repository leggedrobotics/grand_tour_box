# Start by building the docker container
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html#isaac-ros-dev-scripts
# dont know exactly what we did

# TODO inside the docker container!
chown -R admin:admin /workspaces/ros2_ws
chown -R admin:admin /workspaces/bridge_ws
mkdir -p /workspaces/ros2_ws/src
mkdir -p /workspaces/bridge_ws/src

ln -s /home/rsl/git/grand_tour_box/box_drivers_ros2/hdr_camera_driver /workspaces/ros2_ws/src
ln -s /home/rsl/git/grand_tour_box/box_drivers_ros2/isaac_ros_common /workspaces/ros2_ws/src
ln -s /home/rsl/git/grand_tour_box/box_drivers_ros2/rosbag2_composable_recorder /workspaces/ros2_ws/src
ln -s /home/rsl/git/grand_tour_box/box_drivers_ros2/ros1_bridge /workspaces/bridge_ws/src
ln -s /home/rsl/git/grand_tour_box/box_calibration /workspaces/ros2_ws/src


cd /workspaces/ros2_ws; colcon build --symlink-install --packages-select v4l2_camera; source install/setup.bash;
cd /workspaces/ros2_ws; colcon build --symlink-install --packages-select rosbag2_composable_recorder; source install/setup.bash;
cd /workspaces/ros2_ws; source /opt/ros/humble/setup.bash; cd /workspaces/ros2_ws; source install/setup.bash; ros2 launch rosbag2_composable_recorder recorder.launch.py

# Install ROS1
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt -y update
sudo apt install -y ros-noetic-ros-base
sudo apt install -y ros-noetic-catkin
sudo apt install -y ros-noetic-roscpp
sudo apt install -y ros-noetic-roslaunch
sudo apt install -y ros-noetic-rosmsg
sudo apt install -y ros-noetic-std_msgs
sudo apt install -y ros-noetic-rospkg
sudo apt install -y ros-noetic-rostopic

# Build the ros1_bridge in seperate workspace
cd /workspaces/bridge_ws; source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure


# Relevant scripts to understand
- box_drivers_ros2/isaac_ros_common/docker/scripts/entrypoint.py
- box_drivers_ros2/isaac_ros_common/docker/scripts/workspace-entrypoint.sh


# 1. Ideally not install the system on 64GB nvme storage instead use SSD
# 2. install normally docker
# 3. Change then if with the weird installation the docker to use the SSD on the /data 
# Following this here: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms
# It is clear what to do till creating the workspace.
# Then clone the https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/main into the repository
# Follow the readme here and then install the docker container: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/main/scripts


### Helper commands not needed
# Build the workspace without rosbridge
# cd /workspaces/ros2_ws; colcon build --symlink-install --packages-skip ros1_bridge

# Source ROS1 workspace $ then ROS2
# source /opt/ros/noetic/setup.bash
# source /opt/ros/humble/setup.bash

# Buidl rosbridge

# cd /workspaces/ros2_ws; colcon build --packages-select v4l2_camera

# Clean workspace:
# cd /workspaces/ros2_ws; rm -r -f build install log

# Set up common aliases
# echo "alias hdr_start='cd /workspaces/isaac_ros-dev; source /opt/ros/humble/setup.bash; cd /workspaces/isaac_ros-dev; source install/setup.bash; ros2 launch rosbag2_composable_recorder recorder.launch.py'" >> ~/.bashrc
# echo "alias hdr_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select v4l2_camera; source install/setup.bash;'" >> ~/.bashrc
# echo "alias recorder_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select rosbag2_composable_recorder; source install/setup.bash;'" >> ~/.bashrc

# echo "alias ros1_bridge_start='source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics'" >> ~/.bashrc
# echo "alias hdr_encoder='ros2 launch isaac_ros_h264_encoder isaac_ros_h264_encoder.launch.py'" >> ~/.bashrc

# Ensures an interactive shell started with docker run has the aliases set.
# echo "source ~/.bashrc" >> ~/.bash_profile
# echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# echo "source install/setup.bash" >> ~/.bashrc

