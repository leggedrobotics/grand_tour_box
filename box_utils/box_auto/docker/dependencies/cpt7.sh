# CPT7 installation
cd /home/catkin_ws/src/grand_tour_box/box_drivers/novatel_oem7_driver
rosdep install --from-paths src --ignore-src -r -y --reinstall
./build.sh -f