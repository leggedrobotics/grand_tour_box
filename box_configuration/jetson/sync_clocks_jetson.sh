#!/bin/bash

echo "Before:"
date
echo "Restarting ptp4l_mgbe0, ptp4l_mgbe1, phc2sys_mgbe0, phc2sys_mgbe1"
sudo systemctl restart ptp4l_mgbe0
sudo systemctl restart ptp4l_mgbe1
sudo systemctl restart phc2sys_mgbe0
sudo systemctl restart phc2sys_mgbe1
sleep 2
echo "Restarting chrony.service"
sudo systemctl restart chrony.service
echo "After:"
date
