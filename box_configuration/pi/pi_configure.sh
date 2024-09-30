
# Before executing:
# SSH keys are setup
# Correct interfaces: /etc/network/interfaces 
# You can ping google.de
# if not most likely add: sudo ip route add default via 192.168.2.51 dev eth0 onlink

/home/rsl/git/grand_tour_box/box_configuration/general/general_install.sh

sudo apt install netplan.io -y

sudo cp ~/git/grand_tour_box/box_configuration/pi/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml
sudo netplan generate
sudo netplan try
# ACTION hit enter if good

# Set the correct iptables and store it presistently 
# tried to edit sudo vi /etc/rc.local
sudo ip route add default via 192.168.2.51 dev eth0 onlink
sudo apt install iptables-persistent -y
# ACTION ipv4 yes ipv6 no


echo "dtoverlay=pps-gpio,gpiopin=18" >> /boot/firmware/config.txt
echo "dtoverlay=i2c-rtc,pcf85063a,i2c_csi_dsi" >> /boot/firmware/config.txt
echo "dtoverlay=i2c-fan,emc2301,i2c_csi_dsi" >> /boot/firmware/config.txt
echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt

sudo systemctl disable hciuart
sudo dpkg-reconfigure tzdata

ethtool -T eth0
sudo hwclock --show


sudo vi /etc/locale.gen
# ACTION Uncomment correct languages -> Select Europe Zurich
sudo locale-gen
# ACTION Comment in de_CH de_DE en_US en_GB all in UTF standard
sudo update-locale


# Installing PTP and other system services
sudo apt -y install linuxptp

# Disable time sync to internet
sudo timedatectl set-ntp no

sudo cp ~/git/grand_tour_box/box_configuration/pi/phc2sys_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/ptp4l_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_tmux.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_set_fan_speed.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/pigpiod_box.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
sudo cp ~/git/grand_tour_box/box_configuration/pi/clock_info.service /etc/systemd/system/clock_info.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/pi/cleanup_ros_folder.service /etc/systemd/system/cleanup_ros_folder.service

sudo systemctl daemon-reload
sudo systemctl enable cleanup_ros_folder

sudo systemctl enable clock_info
sudo systemctl enable ptp4l_eth0.service
sudo systemctl enable phc2sys_eth0.service
sudo systemctl enable autostart_tmux.service
sudo systemctl enable autostart_set_fan_speed.service
sudo systemctl enable pigpiod_box.service
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

sudo systemctl start ptp4l_eth0.service
sudo systemctl start phc2sys_eth0.service
sudo systemctl start autostart_tmux.service
sudo systemctl start autostart_set_fan_speed.service
sudo systemctl start pigpiod_box.service
sudo systemctl start restart docker

systemctl disable pigpiod.service


systemctl restart ptp4l_eth0.service; sleep 3; systemctl restart phc2sys_eth0.service

# Install for tmux
sudo apt install tmux tmuxp -y

# Install pigpio
sudo apt install python-setuptools python3-setuptools -y

cd ~/git; wget https://github.com/joan2937/pigpio/archive/master.zip
cd ~/git; unzip master.zip
cd pigpio-master
make
sudo make install -j3
sudo ./x_pigpio
sudo pigpiod 
# ACTION check if all pass work



# These will be added by default when installing pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod


# Installing ADIS timestamping - kernel modules add persistent
sudo apt install -y raspberrypi-kernel-headers
make
cd /home/rsl/git/grand_tour_box/box_drivers/adis16475_driver/adis16475_kernel_module
sudo insmod time_stamper_gpio27.ko
# To make the kernel module persistent over reboots:
# copy kernel module to kernel 
sudo cp time_stamper_gpio27.ko /lib/modules/$(uname -r)/
#  re-create the module dependency list 
sudo depmod
# add to modules to load at boot time 
echo 'time_stamper_gpio27' | sudo tee -a /etc/modules
#load kernel module 
modprobe time_stamper_gpio27



# Installing docker
sudo apt -y update
sudo apt -y install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt -y update
sudo apt -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
####


# Install Boxi
cd ~/git/grand_tour_box/box_utils/boxi
sudo apt -y update
sudo apt install python3-pip -y
pip3 install -e ./


cd /home/rsl/git/grand_tour_box/box_configuration/pi/docker
./build.sh
~/git/grand_tour_box/box_configuration/pi/docker/run.sh
# ACTION /home/rsl/git/grand_tour_box/box_configuration/pi/docker/install_manually.sh
# ACTION - open new terminal and do the following - docker commit 12312312 rslethz:noetic-pi-focal
# Done

# Follow the instructions in https://forums.raspberrypi.com/viewtopic.php?t=164868 to get the clock monitoring working.
# In addition the ptp4l and phy2sys create temporary files that are read within the healt_status given taht no systemd access is available within the container.
