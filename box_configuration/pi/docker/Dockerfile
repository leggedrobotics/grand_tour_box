FROM ros:noetic-ros-base-focal

COPY install_inside_container.sh /install_inside_container.sh 
RUN sh -c "chmod +x /install_inside_container.sh"
RUN /bin/bash -c '/install_inside_container.sh'

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh