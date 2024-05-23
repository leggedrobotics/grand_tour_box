#!/bin/bash

# ssh into AP20, start detached rover with IP of the host PC 
ssh root@ap20 'pkill -9 rosrover.bin'

# exit gracefully by returning a status 
exit 0