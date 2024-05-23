#!/bin/bash

# loop until ssh is available
until nc -vzw 2 ap20 22; do sleep 2; done
echo "AP20 could be reached, next try to ssh"

ssh root@ap20 'pkill -9 rosrover.bin'

# ssh into AP20, start detached rover with IP of the host PC 
ssh root@ap20 'nohup start_rosrover.bash 2>&1 &'

# exit gracefully by returning a status 
exit 0