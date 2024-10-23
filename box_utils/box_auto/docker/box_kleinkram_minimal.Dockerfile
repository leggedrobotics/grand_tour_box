FROM ros:noetic-ros-base-focal

RUN pwd
RUN pwd
COPY dependencies/minimal.sh /minimal.sh
RUN chmod +x /minimal.sh
RUN /bin/bash -c '/minimal.sh'

COPY entrypoint_kleinkram_minimal.sh /entrypoint_kleinkram_minimal.sh
RUN chmod +x /entrypoint_kleinkram_minimal.sh

ENTRYPOINT ["/entrypoint_kleinkram_minimal.sh"]