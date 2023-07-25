#!/bin/bash

# Constants
JETSON_IP="192.168.2.51"
NUC_IP="192.168.2.56"
OPC_IP="192.168.2.151"

# Wait for Jetson to be ready
echo "Waiting for Jetson"
until ping -c1 $JETSON_IP >/dev/null 2>&1; do :; done
echo "Found Jetson"

# Wait for the rosmaster on the Jetson to be ready
echo "Waiting for Rosmaster"
until rosnode ping -c1 "rosout" >/dev/null 2>&1; do :; done
echo "Found rosmaster"