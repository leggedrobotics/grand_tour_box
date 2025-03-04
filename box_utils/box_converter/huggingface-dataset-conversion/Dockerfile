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
ADD git@github.com:leggedrobotics/grand_tour_box.git /catkin_ws/src/
ADD git@github.com:ros-drivers/gps_umd.git /catkin_ws/src/
ADD git@github.com:ANYbotics/signal_logger.git /catkin_ws/src/
ADD git@github.com:GAVLab/gavlab_msgs.git /catkin_ws/src/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin build \
    anymal_msgs \
    gps_common \
    signal_logger_msgs \
    gnss_msgs \
    && :

# entrypoint
WORKDIR /app
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
