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

# installing ros packages
WORKDIR /catkin_ws/src
RUN git clone https://github.com/leggedrobotics/tf_bag.git
RUN git clone https://github.com/leggedrobotics/physical_terrain_parameter_learning.git
RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin build

WORKDIR /app
COPY requirements-docker.txt requirements-docker.txt
RUN pip install -r requirements-docker.txt

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
