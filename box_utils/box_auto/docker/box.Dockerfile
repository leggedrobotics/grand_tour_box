FROM nvidia/cuda:12.5.1-base-ubuntu20.04

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility

RUN echo "Europe/Zurich" > /etc/localtime ; echo "CUDA Version 12.5.1" > /usr/local/cuda/version.txt

COPY dependencies/zed.sh /zed.sh
RUN sh -c "chmod +x /zed.sh"
RUN /bin/bash -c '/zed.sh'

COPY dependencies/general.sh /general.sh
RUN sh -c "chmod +x /general.sh"
RUN /bin/bash -c '/general.sh'

COPY dependencies/ros.sh /ros.sh
RUN sh -c "chmod +x /ros.sh"
RUN /bin/bash -c '/ros.sh'

COPY dependencies/open_cv.sh /open_cv.sh
RUN sh -c "chmod +x /open_cv.sh"
RUN /bin/bash -c '/open_cv.sh'

RUN echo Test
COPY dependencies/grand_tour.sh /grand_tour.sh
RUN sh -c "chmod +x /grand_tour.sh"
RUN --mount=type=ssh /bin/bash -c '/grand_tour.sh'

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]