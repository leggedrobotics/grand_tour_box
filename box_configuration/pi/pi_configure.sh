sudo apt-get -y install linuxptp

sudo cp ~/git/grand_tour_box/box_configuration/pi/phc2sys_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/ptp4l_eth0.service /lib/systemd/system/

sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_tmux.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/autostart_set_fan_speed.service /lib/systemd/system/


sudo systemctl daemon-reload
systemctl start phc2sys_eth0.service
systemctl start ptp4l_eth0.service
systemctl start autostart_tmux.service
systemctl start autostart_set_fan_speed.service

systemctl enable phc2sys_eth0.service
systemctl enable ptp4l_eth0.service
systemctl enable autostart_tmux.service
systemctl enable autostart_set_fan_speed.service


# systemctl status phc2sys_eth0.service
# systemctl status ptp4l_eth0.service
# systemctl restart phc2sys_eth0.service; systemctl restart ptp4l_eth0.service

# Disable time sync to internet
sudo timedatectl set-ntp no


# Install pigpio
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install

# These will be added by default when installing pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod




# Installing fkie multi master
pip install grpcio-tools -v
 # Push with git history boxi once

catkin build fkie_multimaster