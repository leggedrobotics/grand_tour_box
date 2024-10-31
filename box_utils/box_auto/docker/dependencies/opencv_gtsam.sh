mkdir -p /home/opencv_gtsam_ws/src
source /opt/ros/noetic/setup.bash
cd /home/opencv_gtsam_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


cd /home/opencv_gtsam_ws/src; git clone https://github.com/catkin/catkin_simple.git --depth 1
cd /home/opencv_gtsam_ws/src; git clone https://github.com/leggedrobotics/gtsam_catkin.git --depth 1

cd /home/opencv_gtsam_ws


# Install OpenCV optionally
cd /home/opencv_gtsam_ws/src; git clone https://github.com/ori-drs/opencv_catkin.git --depth 1
catkin build opencv_catkin --cmake-args -DCUDA_ARCH_BIN=8.6

catkin build gtsam_catkin
echo "source /home/opencv_gtsam_ws/devel/setup.bash" >> /root/.bashrc