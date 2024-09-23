# Boxi

## Installation
From this directory run `pip install -e .` to install boxi 
in editable form. This is the preferred method of installation
that introduces minimal changes to `jetson` and the `nuc` when
running `boxi push`

## Apps
### Boxi export_imu_log
#### Usage
After a data collection run, extract a RAW_IMU_ASCII txt file using
the Novatel Convert app, available on the Ubuntu Novatel Application Suite
`boxi export_imu_log --imu_ascii_file/-i PATH_TO_RAW_IMU_ASCII.txt
--output/-o OUTPUT_ROSBAG_PATH.bag`

