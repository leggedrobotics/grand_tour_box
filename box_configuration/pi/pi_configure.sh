# NR 1 configure correctly the interfaces: /etc/network/interfaces 
# We had some problems with ip route show: sudo ip route del default via 192.168.2.151 dev eth0 -> the upper command fixed it
ssh-add /home/rsl/.ssh/id_rsa
# sudo ip route add default via 192.168.2.51 dev eth0 onlink
sudo apt install netplan.io -y

sudo cp ~/git/grand_tour_box/box_configuration/pi/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml
sudo netplan generate
sudo netplan apply


echo "dtoverlay=pps-gpio,gpiopin=18" >> /boot/firmware/config.txt
echo "dtoverlay=i2c-rtc,pcf85063a,i2c_csi_dsi" >> /boot/firmware/config.txt
echo "dtoverlay=i2c-fan,emc2301,i2c_csi_dsi" >> /boot/firmware/config.txt
echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt

sudo systemctl disable hciuart
sudo dpkg-reconfigure tzdata

ethtool -T eth0
sudo hwclock --show


# Disable DHCP suggestion (not tested)
# https://superuser.com/questions/547114/all-statically-assigned-addresses-but-dhcpcd-still-runs
# sudo update-rc.d -f dhcpd remove
# Reenable DHCP
# sudo update-rc.d dhcpd defaults
###################

sudo vi /etc/locale.gen
# uncomment correct languages
sudo locale-gen
sudo update-locale


# Installing PTP and other system services
sudo apt-get -y install linuxptp

# Disable time sync to internet
sudo timedatectl set-ntp no

sudo cp ~/git/grand_tour_box/box_configuration/pi/phc2sys_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/ptp4l_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_tmux.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_set_fan_speed.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/pigpiod.service /lib/systemd/system/

sudo systemctl daemon-reload
systemctl start ptp4l_eth0.service
systemctl start phc2sys_eth0.service
systemctl start autostart_tmux.service
systemctl start autostart_set_fan_speed.service
systemctl start pigpiod.service
systemctl start restart docker

systemctl enable ptp4l_eth0.service
systemctl enable phc2sys_eth0.service
systemctl enable autostart_tmux.service
systemctl enable autostart_set_fan_speed.service
systemctl enable pigpiod.service
systemctl enable docker.service
systemctl enable containerd.service


systemctl restart ptp4l_eth0.service; sleep 3; systemctl restart phc2sys_eth0.service


# Install for tmux
sudo apt install tmux tmuxp -y


# Install pigpio
cd ~/git
sudo apt install python-setuptools python3-setuptools -y
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install -j3
sudo ./x_pigpio
sudo pigpiod 

# These will be added by default when installing pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod


# Installing ADIS timestamping
sudo apt install -y raspberrypi-kernel-headers
cd /home/rsl/git/grand_tour_box/box_drivers/adis16475_driver/adis16475_kernel_module
make
sudo insmod time_stamper_adis.ko


# Installing docker
sudo apt-get -y update
sudo apt-get -y install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get -y update
sudo apt-get -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
####



# Install Boxi
cd ~/git/grand_tour_box/box_utils/boxi
sudo apt install python3-pip -y
pip3 install -e ./



cd /home/rsl/git/grand_tour_box/box_configuration/pi/docker
./build.sh
~/git/grand_tour_box/box_configuration/pi/docker/run.sh
# /home/rsl/git/grand_tour_box/box_configuration/pi/docker/install_manually.sh
# docker commit 12312312 leggedrobotics:noetic-pi-focal
# Done