# Following this https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
# locale  # check for UTF-8
# apt update -y 
# apt install locales -y
# locale-gen en_US en_US.UTF-8
# update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8
# locale  # verify settings

# apt -y install software-properties-common
# add-apt-repository universe -y 

# apt update && apt install curl -y
# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# apt update -y 

# apt install -y \
#   python3-flake8-docstrings \
#   python3-pip \
#   python3-pytest-cov \
#   ros-dev-tools


# python3 -m pip install -U \
#    flake8-blind-except \
#    flake8-builtins \
#    flake8-class-newline \
#    flake8-comprehensions \
#    flake8-deprecated \
#    flake8-import-order \
#    flake8-quotes \
#    "pytest>=5.3" \
#    pytest-repeat \
#    pytest-rerunfailures

# mkdir -p ~/ros2_humble/src
# cd ~/ros2_humble
# vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# rosdep init
# rosdep update 
# rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

mkdir -p /home/catkin_ws/src
cd /home/catkin_ws
catkin init

source /opt/ros/noetic/setup.bash
cd /home/catkin_ws/src; git clone --shallow-submodules git@github.com:leggedrobotics/grand_tour_box.git --depth 1
pip install rosbags

catkin build box_calibration
