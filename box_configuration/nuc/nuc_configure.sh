cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan generate
sudo netplan apply


cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/autostart_tmux.service /etc/systemd/system/autostart_tmux.service
cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/phc2sys_NIC.service /lib/systemd/system/phc2sys_NIC.service
cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/phc2sys_system.service /lib/systemd/system/phc2sys_system.service
cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/ptp4l_enp45s0.service /lib/systemd/system/ptp4l_enp45s0.service
cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/ptp4l_enp46s0.service /lib/systemd/system/ptp4l_enp46s0.service
sudo systemctl daemon-reload
sudo enable autostart_tmux
sudo enable phc2sys_NIC
sudo enable phc2sys_system
sudo enable ptp4l_enp45s0
sudo enable ptp4l_enp46s0

cp /home/rsl/catkin_ws/src/grand_tour_box/box_configuration/nuc/99-GMSL-kit.rules /etc/udev/rules.d/
 udevadm control --reload && udevadm trigger