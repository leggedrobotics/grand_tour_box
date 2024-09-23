# Boxi

## Installation
From this directory run `pip install -e .` to install boxi 
in editable form. This is the preferred method of installation
that introduces minimal changes to `jetson` and the `nuc` when
running `boxi push`

## Apps
### Boxi export_raw_imu_bag
#### Usage
After a data collection run, extract a RAW_IMU_ASCII txt file using
the Novatel Convert app, available on the Ubuntu Novatel Application Suite
`boxi export_raw_imu_bag --imu_ascii_file/-i PATH_TO_RAW_IMU_ASCII.txt
--output/-o OUTPUT_ROSBAG_PATH.bag`

### Boxi export_gps_gt_trajectory_bag
#### Usage
Launch the **Inertial Explorer** app in Windows on the Boxi OPC. 
After start up, start a new project using the Project Wizard.
Select a name and location to save the project, then proceed with the Project Wizard
window to load the CPT7 .LOG file. [TODO] Insert link to the instructions on running
offline optimization of the trajectories. Once done optimizing the GPS trajectories,
export a trajectory .txt file using the profile: **GrandTour GroundTruth**.
Back over in Ubuntu, run `boxi export_gps_gt_trajectory_bag --gps_file/-g PATH_TO_GPS_TRAJECTORY.txt
--output/-o OUTPUT_ROSBAG_PATH.bag`
