FROM ros:noetic-perception-focal

COPY ./docker/color_correction/entrypoint.sh /entrypoint.sh
COPY ./docker/color_correction/color_correction.py /color_correction.py
RUN chmod +x /entrypoint.sh
RUN mkdir -p /home/catkin_ws/src
RUN apt-get update && apt install -y python3-catkin-tools git libyaml-cpp-dev libtool
RUN source /opt/ros/noetic/setup.sh; cd /home/catkin_ws; catkin init; catkin config --extend /opt/ros/noetic; catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
RUN cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/raw_image_pipeline.git
RUN cd /home/catkin_ws/src; git clone https://github.com/leggedrobotics/pybind11_catkin.git
RUN cd /home/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git
RUN cd /home/catkin_ws/src; git clone https://github.com/ethz-asl/glog_catkin.git
RUN source /opt/ros/noetic/setup.sh; cd /home/catkin_ws; catkin build raw_image_pipeline_ros
RUN source /opt/ros/noetic/setup.sh; cd /home/catkin_ws; catkin build raw_image_pipeline_python

ENTRYPOINT ["/entrypoint.sh"]