FROM nvidia/cuda:12.5.1-base-ubuntu20.04

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


COPY install_inside_container.sh /install_inside_container.sh 
COPY entrypoint.sh /entrypoint.sh

#RUN sh -c "chmod +x /install_inside_container.sh"
#RUN /bin/bash -c '/install_inside_container.sh'

#RUN chmod +x /entrypoint.sh