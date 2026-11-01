FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-virtualenv \
    && rm -rf /var/lib/apt/lists/*

RUN virtualenv /opt/venv -ppython3
ENV PATH=/opt/venv/bin:$PATH

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbag \
    ros-$ROS_DISTRO-sensor-msgs \
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
# Fail at build time if the Gen1 files selected by the deployed entry point are absent.
RUN test -f /models/ego_blur_face_gen1.jit -o -f /models/ego_blur_face.jit && \
    test -f /models/ego_blur_lp_gen1.jit -o -f /models/ego_blur_lp.jit

# python deps
WORKDIR /app
COPY requirements-docker.txt requirements-docker.txt
RUN pip install -r requirements-docker.txt

COPY ./anonymization.py /app/anonymization.py
COPY ./bag_anonymization.py ./image_codec.py /app/

# entrypoint
WORKDIR /app
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
