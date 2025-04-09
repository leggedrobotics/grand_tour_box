FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-virtualenv \
    && rm -rf /var/lib/apt/lists/*

RUN virtualenv /opt/venv -ppython3
ENV PATH=/opt/venv/bin:$PATH

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-tf \
    ffmpeg \
    libsm6 \
    libxext6 \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    pycryptodomex \
    pyyaml \
    python-gnupg \
    rospkg \
    catkin-tools \
    osrf-pycommon \
    empy==3.3.4 \
    && :

# install and setup catkin_ws
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN catkin init

# python deps
WORKDIR /app
COPY requirements-docker.txt requirements-docker.txt
RUN pip install -r requirements-docker.txt

# installing ros packages
WORKDIR /catkin_ws/src
ADD git@github.com:leggedrobotics/grand_tour_box.git grand_tour_box
ADD git@github.com:ros-drivers/gps_umd.git gps_umd
ADD git@github.com:ANYbotics/signal_logger.git signal_logger
RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin build \
    anymal_msgs \
    gps_common \
    signal_logger_msgs \
    gnss_msgs \
    && :

# For debugging in vs code in docker

# search for packages --> find /catkin_ws -type d -name gnss_msgs | grep dist-packages
# test symbolic linke --> find /catkin_ws -type l -name gnss_msgs | grep dist-packages

# ## Create symlinks for ROS packages in the virtualenv
RUN VENV_SITE_PACKAGES=/opt/venv/lib/python3.8/site-packages && \
ln -s /opt/ros/noetic/lib/python3/dist-packages/sensor_msgs $VENV_SITE_PACKAGES/ && \
ln -s /opt/ros/noetic/lib/python3/dist-packages/std_msgs $VENV_SITE_PACKAGES/ && \
ln -s /opt/ros/noetic/lib/python3/dist-packages/genpy $VENV_SITE_PACKAGES/ && \
ln -s /opt/ros/noetic/lib/python3/dist-packages/rospy $VENV_SITE_PACKAGES/ && \
ln -s /opt/ros/noetic/lib/python3/dist-packages/roslib $VENV_SITE_PACKAGES/ && \
ln -s /catkin_ws/devel/lib/python3/dist-packages/anymal_msgs /opt/venv/lib/python3.8/site-packages/ && \
ln -s /catkin_ws/devel/lib/python3/dist-packages/gnss_msgs /opt/venv/lib/python3.8/site-packages/ && \
ln -s /catkin_ws/devel/lib/python3/dist-packages/gps_common /opt/venv/lib/python3.8/site-packages/

# entrypoint
WORKDIR /app
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
