mkdir -p /home/opencv_ws/src
source /opt/ros/noetic/setup.bash
cd /home/opencv_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd /home/opencv_ws/src; git clone https://github.com/ori-drs/opencv_catkin.git
cd /home/opencv_ws/src; git clone https://github.com/catkin/catkin_simple.git
catkin build opencv_catkin --cmake-args -DCUDA_ARCH_BIN=8.6
