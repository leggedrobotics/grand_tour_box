FROM ros:noetic-perception-focal

COPY ./docker/topic_freq/entrypoint.sh /entrypoint.sh
COPY ./docker/topic_freq/topic_freq.py /topic_freq.py
RUN chmod +x /entrypoint.sh
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install matplotlib
ENTRYPOINT ["/entrypoint.sh"]