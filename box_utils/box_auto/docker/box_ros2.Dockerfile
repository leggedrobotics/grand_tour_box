FROM osrf/ros:humble-desktop
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend=noninteractive

COPY dependencies/general.sh /general.sh
RUN sh -c "chmod +x /general.sh"
RUN /bin/bash -c '/general.sh'

COPY dependencies/ros2.sh /ros2.sh
RUN sh -c "chmod +x /ros2.sh"
RUN --mount=type=ssh /bin/bash -c '/ros2.sh'

COPY entrypoint_ros2.sh /home/rsl/entrypoint_ros2.sh
RUN chmod +x /home/rsl/entrypoint_ros2.sh
RUN chown rsl:rsl -R /home/rsl

USER rsl
WORKDIR /home/rsl

ENTRYPOINT ["/home/rsl/entrypoint_ros2.sh"]