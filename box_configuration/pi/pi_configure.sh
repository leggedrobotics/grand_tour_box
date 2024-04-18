sudo apt-get -y install linuxptp

sudo cp ~/git/grand_tour_box/box_configuration/pi/phc2sys_eth0.service /lib/systemd/system/
sudo cp ~/git/grand_tour_box/box_configuration/pi/ptp4l_eth0.service /lib/systemd/system/
sudo systemctl daemon-reload
systemctl start phc2sys_eth0.service
systemctl start ptp4l_eth0.service
# systemctl status phc2sys_eth0.service
# systemctl status ptp4l_eth0.service
# systemctl restart phc2sys_eth0.service; systemctl restart ptp4l_eth0.service

# Disable time sync to internet
sudo timedatectl set-ntp no