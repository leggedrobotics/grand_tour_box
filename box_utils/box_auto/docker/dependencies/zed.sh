#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <CUDA_MAJOR> <CUDA_MINOR>"
    exit 1
fi

CUDA_MAJOR=$1
CUDA_MINOR=$2
CUDA_VERSION="${CUDA_MAJOR}${CUDA_MINOR}"

# Here we use the ZED SDK 4.1 for Ubuntu 20.04 and complied for CUDA 12.1 instead of 12.5.1
apt-get update -y
apt-get install --no-install-recommends lsb-release wget less udev sudo zstd build-essential cmake python3 python3-pip libpng-dev libgomp1 -y
python3 -m pip install numpy opencv-python
cd /
wget -q -O ZED_SDK_Linux_Ubuntu.run "https://download.stereolabs.com/zedsdk/4.2/cu${CUDA_MAJOR}/ubuntu20"; chmod +x ZED_SDK_Linux_Ubuntu.run 
./ZED_SDK_Linux_Ubuntu.run  -- silent
ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so 
rm ZED_SDK_Linux_Ubuntu.run 
rm -rf /var/lib/apt/lists/*
