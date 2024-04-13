#!/bin/bash

# .bashrc
echo '' >> ~/.bashrc
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source catkin_ws/devel/setup.bash' >> ~/.bashrc
echo 'source git/grand_tour_box/box_utils/box_setup/alias/alias.sh' >> ~/.bashrc

# netplan
cp /home/rsl/git/grand_tour_box/box_configuration/nuc/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan generate
sudo netplan apply

# autostart tmux & ptp
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/phc2sys_enp45s0.service /lib/systemd/system/phc2sys_enp45s0.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/phc2sys_enp46s0.service /lib/systemd/system/phc2sys_enp46s0.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/ptp4l_enp45s0.service /lib/systemd/system/ptp4l_enp45s0.service
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/ptp4l_enp46s0.service /lib/systemd/system/ptp4l_enp46s0.service
sudo systemctl daemon-reload
sudo systemctl enable autostart_tmux
sudo systemctl enable phc2sys_enp45s0
sudo systemctl enable phc2sys_enp46s0
sudo systemctl enable ptp4l_enp45s0
sudo systemctl enable ptp4l_enp46s0

cp /home/rsl/git/grand_tour_box/box_configuration/nuc/99-GMSL-kit.rules /etc/udev/rules.d/
cp /home/rsl/git/grand_tour_box/box_configuration/nuc/99-hdr-camera.rules /etc/udev/rules.d/
 udevadm control --reload && udevadm trigger