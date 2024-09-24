FROM osrf/ros:humble-desktop

RUN apt update -y
RUN apt install -y wget
RUN apt install python3-pip -y


RUN apt install -y git-all
RUN mkdir -p /root/.ssh
RUN ssh-keyscan github.com 2> /dev/null >> /root/.ssh/known_hosts

RUN mkdir -p /home/catkin_ws/src; 
RUN --mount=type=ssh /bin/bash -c 'cd /home/catkin_ws/src; git clone git@github.com:leggedrobotics/grand_tour_box.git'
RUN cd /bin; wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.47/mcap-linux-amd64; chmod +x mcap-linux-amd64; cp mcap-linux-amd64 mcap

ENTRYPOINT ["/ros_entrypoint.sh"]