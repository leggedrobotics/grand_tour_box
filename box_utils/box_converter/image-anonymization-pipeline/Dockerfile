FROM ros:noetic

ARG KLEINKRAM_CONFIG

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

# copy the model files
COPY ./models /models

# python deps
WORKDIR /app
COPY requirements-docker.txt requirements-docker.txt
RUN pip install -r requirements-docker.txt

COPY ./anonymization.py /app/anonymization.py

# entrypoint
WORKDIR /app
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
