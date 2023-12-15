#!/bin/bash

# ssh into AP20, start detached rover with IP of the host PC 
ssh root@ap20 'pkill rover'

# exit gracefully by returning a status 
exit 0