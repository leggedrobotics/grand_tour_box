
cd /home/rsl/git/pigpio-master
make
sudo make install
# Verify working
./x_pigpio

# Installing fkie multi master
cd /home/rsl/git/grand_tour_box/box_drivers
mkdir -p /home/rsl/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd /home/rsl/catkin_ws
ln -s ~/git/grand_tour_box ~/catkin_ws/src/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build fkie_multimaster

cd /home/rsl/git/grand_tour_box/box_drivers/adis16475_driver/adis16475_driver
mkdir build && cd cmake .. ; sudo make install

cd /home/rsl/catkin_ws; catkin build launch_pi

# Also ensure to add the correct source script to /etc/bash.bashrc
sudo su
echo "source /opt/ros/noetic/setup.bash " >> /etc/bash.bashrc 
echo "source /home/rsl/catkin_ws/devel/setup.bash " >> /etc/bash.bashrc 
echo "export HOSTNAME=pi" >> /etc/bash.bashrc 
echo "export DISPLAY=:0.0" >> /etc/bash.bashrc 
echo "export ROS_MASTER_URI=http://192.168.2.57:11311" >> /etc/bash.bashrc 
echo "export ROS_IP=192.168.2.57" >> /etc/bash.bashrc 
echo 'export PS1="\[\033[1;34m\](docker) \[\033[1;32m\]\u@\h \[\033[1;34m\]\w \$\[\033[0m\] "' >> /etc/bash.bashrc