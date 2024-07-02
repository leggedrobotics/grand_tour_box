FROM ros:noetic-perception-focal

COPY ./docker/rectify/entrypoint.sh /entrypoint.sh
COPY ./docker/rectify/rectify.py /rectify.py
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]