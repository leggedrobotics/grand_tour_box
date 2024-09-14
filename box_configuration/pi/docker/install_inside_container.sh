
sudo apt-get update -y
sudo apt-get install -y build-essential
sudo apt install -y python-setuptools python3-setuptools

sudo apt-get install vim -y
sudo apt-get install ros-noetic-catkin python3-catkin-tools -y
sudo apt-get update -y
sudo apt install python3-pip -y
sudo apt install python3-grpc-tools -y
pip3 install ruamel.yaml==0.15.89
pip3 install psutil
sudo apt install screen -y
sudo apt-get install figlet -y
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update -y

# Dependencies
sudo apt install libusb-1.0-0-dev -y
sudo apt install libavahi-client-dev -y
sudo apt install flex bison -y
sudo apt install libaio-dev -y
sudo apt install libxml2-dev -y
sudo apt install git -y

cd /home/rsl/git
git clone https://github.com/analogdevicesinc/libiio.git -b v0.25 --depth=1
cd libiio
mkdir build && cd build
cmake ..
make -j
sudo make install

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/rsl/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "PS1=\"\[\e[1;32m\](docker)\[\e[0;37m\]\[\e[1;34m\]:\[\e[0;33m\]\w\[\e[0m\] \"" >> ~/.bashrc

sudo apt install wget -y