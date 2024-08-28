#!/bin/bash

echo "Before:"
date
echo "Restarting ptp4l_mgbe0, ptp4l_mgbe1, phc2sys_mgbe0, phc2sys_mgbe1, box_chrony"
sudo systemctl restart ptp4l_mgbe0
sleep 1.0
sudo systemctl restart phc2sys_mgbe0
sleep 2.0
echo "Restarting chrony.service"
sudo systemctl restart box_chrony
sleep 5
echo "After:"
date
