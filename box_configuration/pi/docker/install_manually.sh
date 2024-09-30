cd /home/rsl/git/pigpio-master
make
sudo make install
# Verify working
./x_pigpio

# Installing fkie multi master
mkdir -p /home/rsl/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd /home/rsl/catkin_ws
ln -s ~/git/grand_tour_box ~/catkin_ws/src/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Also ensure to add the correct source script to /etc/bash.bashrc
sudo su
echo "source /opt/ros/noetic/setup.bash " >> /etc/bash.bashrc 
echo "source /home/rsl/catkin_ws/devel/setup.bash " >> /etc/bash.bashrc
echo "source /home/rsl/git/grand_tour_box/box_configuration/pi/alias.sh" >> /etc/bash.bashrc

cd /home/rsl/catkin_ws/src/grand_tour_box/box_drivers
git clone git@github.com:leggedrobotics/multimaster_fkie.git -b grandTour --depth=1
rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie -y
git config --global --add safe.directory /home/rsl/git/grand_tour_box/box_drivers/multimaster_fkie

cd /home/rsl/catkin_ws; catkin build launch_pi
