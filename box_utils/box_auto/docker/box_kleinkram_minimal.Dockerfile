FROM ros:noetic-ros-base-focal

COPY dependencies/minimal.sh /minimal.sh
RUN chmod +x /minimal.sh
RUN --mount=type=ssh /bin/bash -c '/minimal.sh'

COPY entrypoint_kleinkram_minimal.sh /entrypoint_kleinkram_minimal.sh
RUN chmod +x /entrypoint_kleinkram_minimal.sh

COPY entrypoint_minimal.sh /entrypoint_minimal.sh
RUN chmod +x /entrypoint_minimal.sh

ENTRYPOINT ["/entrypoint_kleinkram_minimal.sh"]