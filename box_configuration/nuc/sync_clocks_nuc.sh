#!/bin/bash

echo "Before:"
date
echo "Restarting ptp4l_enp45s0, ptp4l_enp46s0, phc2sys_enp45s0, phc2sys_enp46s0"
sudo systemctl restart ptp4l_enp45s0
sleep 1
sudo systemctl restart ptp4l_enp46s0
sleep 1
sudo systemctl restart phc2sys_enp45s0.service
sleep 5
echo "After:"
date