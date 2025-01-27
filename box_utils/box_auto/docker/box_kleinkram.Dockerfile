

FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu20.04 AS builder
# FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu20.04 AS builder 

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility

# Token for creating github issues from inside the container.
ENV GITHUB_TOKEN=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

RUN echo "Europe/Zurich" > /etc/localtime ; echo "CUDA Version 12.2.2" > /usr/local/cuda/version.txt

COPY dependencies/zed.sh /zed.sh
RUN sh -c "chmod +x /zed.sh"
RUN /bin/bash -c '/zed.sh 12 2'

COPY dependencies/general.sh /general.sh
RUN sh -c "chmod +x /general.sh"
RUN /bin/bash -c '/general.sh'

COPY dependencies/ros.sh /ros.sh
RUN sh -c "chmod +x /ros.sh"
RUN /bin/bash -c '/ros.sh'

COPY dependencies/opencv_gtsam.sh /opencv_gtsam.sh
RUN sh -c "chmod +x /opencv_gtsam.sh"
RUN /bin/bash -c '/opencv_gtsam.sh'

COPY dependencies/grand_tour.sh /grand_tour.sh
RUN sh -c "chmod +x /grand_tour.sh"
RUN --mount=type=ssh /bin/bash -c '/grand_tour.sh'

COPY entrypoint_kleinkram.sh /entrypoint_kleinkram.sh
RUN chmod +x /entrypoint_kleinkram.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint_kleinkram.sh"]