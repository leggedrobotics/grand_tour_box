apt update -y
apt install -y figlet
apt install -y git-all
mkdir -p /root/.ssh
ssh-keyscan github.com >> /root/.ssh/known_hosts
ssh-keyscan bitbucket.org >> /root/.ssh/known_hosts


apt install -y python3-catkin-tools
mkdir -p /home/catkin_ws/src

source /opt/ros/noetic/setup.bash
cd /home/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd /home/catkin_ws/src; git clone git@github.com:leggedrobotics/grand_tour_box.git --depth 1
cd /home/catkin_ws/src/grand_tour_box; git submodule init box_applications/tf_bag
cd /home/catkin_ws/src/grand_tour_box; git submodule update box_applications/tf_bag

cd /home/catkin_ws; catkin build ap20_driver_ros

rosdep update
cd /home/catkin_ws/src/grand_tour_box/box_applications/tf_bag
rosdep install --from-paths . --ignore-src -r -y --reinstall

# cv_bridge 
apt install -q -y ros-noetic-cv-bridge 
apt install -q -y python3-opencv
apt install -q -y python3-pip
pip install matplotlib
pip install kleinkram==0.43.2
