#!/bin/bash

# ssh into AP20, start detached rover with IP of the host PC 
ssh root@ap20 'echo 192.168.2.51 | nohup ap20_start/gt_box_rover.bash 2>&1 &'

# exit gracefully by returning a status 
exit 0