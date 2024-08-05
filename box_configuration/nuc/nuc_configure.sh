#!/bin/bash

rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/nuc/.bashrc ~/

# .bashrc
#echo '' >> ~/.bashrc
# Prepend to bashrc so that they still run when we are ssh'd into the PCs (when the interactive session is not started).
#echo 'source /opt/ros/noetic/setup.bash' | cat - ~/.bashrc > temp && mv temp ~/.bashrc
#echo 'source /home/rsl/catkin_ws/devel/setup.bash' | cat - ~/.bashrc > temp && mv temp ~/.bashrc
#echo 'source /home/rsl/git/grand_tour_box/box_configuration/nuc/alias.sh' | cat - ~/.bashrc > temp && mv temp ~/.bashrc

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

# sync clocks script
sudo cp /home/rsl/git/grand_tour_box/box_configuration/nuc/sync_clocks_nuc.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/sync_clocks_nuc.sh

cp /home/rsl/git/grand_tour_box/box_configuration/nuc/99-GMSL-kit.rules /etc/udev/rules.d/
cp /home/rsl/git/grand_tour_box/box_configuration/nuc/99-hdr-camera.rules /etc/udev/rules.d/
 udevadm control --reload && udevadm trigger


cd git; git clone git@github.com:mmattamala/pybind11_catkin.git
 ln -s ~/git/pybind11_catkin ~/catkin_ws/src/



 # Install alphasense drivers
# Add the Sevensense PGP key to make this machine trust Sevensense's packages.
curl -Ls http://deb.7sr.ch/pubkey.gpg | sudo gpg --dearmor -o /usr/share/keyrings/deb-7sr-ch-keyring.gpg

# Add the Sevensense APT repository to the list of known sources.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] http://deb.7sr.ch/alphasense/stable $(lsb_release -cs) main" \
          | sudo tee /etc/apt/sources.list.d/sevensense.list

# Install the Alphasense driver.
sudo apt update -y
sudo apt install alphasense-driver-core alphasense-viewer alphasense-firmware ros-noetic-alphasense-driver-ros ros-noetic-alphasense-driver -y

# Set the maximum socket buffer size
sudo sysctl -w net.core.rmem_max=11145728
echo -e "net.core.rmem_max=11145728" | sudo tee /etc/sysctl.d/90-increase-network-buffers.conf

# CPT7 installation
cd /home/rsl/git/grand_tour_box/box_drivers/novatel_oem7_driver
rosdep install --from-paths src --ignore-src -r -y --reinstall
./build.sh -f
