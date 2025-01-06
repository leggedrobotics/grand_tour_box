
FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu20.04 AS builder
# FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu20.04 AS builder 

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility


COPY dependencies/general.sh /general.sh
RUN sh -c "chmod +x /general.sh"
RUN /bin/bash -c '/general.sh'

COPY dependencies/ros.sh /ros.sh
RUN sh -c "chmod +x /ros.sh"
RUN /bin/bash -c '/ros.sh'

COPY dependencies/ros2_bridge.sh /ros2_bridge.sh
RUN sh -c "chmod +x /ros2_bridge.sh"
RUN --mount=type=ssh /bin/bash -c '/ros2_bridge.sh'


COPY entrypoint_ros2_bridge.sh /entrypoint_ros2_bridge.sh
RUN chmod +x /entrypoint_ros2_bridge.sh

ENTRYPOINT ["/entrypoint_ros2_bridge.sh"]