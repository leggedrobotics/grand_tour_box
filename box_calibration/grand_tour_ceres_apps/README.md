# Grand Tour Ceres Apps

[![Build Status](https://github.com/fulkast/grand_tour_ceres_apps/actions/workflows/ci.yaml/badge.svg)](https://github.com/fulkast/grand_tour_ceres_apps/actions)

This project holds several apps used in calibrating sub-components of the GTBox

## Prerequisites

Make sure you have the following tools installed on your system:

- **CMake** (version 3.10 or higher)
- **C++ Compiler** (GCC, Clang, or MSVC)
- **Git** (optional but recommended)

### Installing Dependencies

#### Ubuntu/Debian

You can install all the required dependencies using the following `apt` commands:

```bash
sudo apt-get update
sudo apt-get install -y \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev
```

### Catkin Build
```bash
catkin build grand_tour_ceres_apps
```

### Running the camera camera calibration
First start up the detectors and the viewers. Note that the detectors are to be separately called from
the NUC and the Jetson, respectively. The viewer and the calibration node are run from the OPC.
```bash
roslaunch grand_tour_camera_detectors detectors_on_nuc_generic.launch use_april_grid:=true
roslaunch grand_tour_camera_detectors detectors_on_jetson_generic.launch use_april_grid:=true
roslaunch grand_tour_calibration_viewers all_cameras.launch

ROS_NAMESPACE=/gt_box rosrun grand_tour_ceres_apps camera_camera_online_calibration 
```
Running the node above in the `/gt_box` namespace is required for smoothly interfacing with the detector and viewer nodes

### Available services
#### gt_box/camera_camera_online_calibration/finalize
This service notifies the calibrator to stop receiving new detections. This also makes the calibrator node notify the
detector nodes to stop recording calibration data. 
Lastly, and aptly, this finalizes the calibration meaning that the solver performs and exhaustive optimization run,
then writes the camera calibrations to 
`box_calibration/box_calibration/calibration/raw_calibration_output/cameras-intrinsics-extrinsics_latest.yaml`

### Fixes in progress
Sometimes the front and right HDR cameras are swapped in their hardware mapping. As the calibrator node currently heavily
relies on the initial guess of the camera transforms, this means that the extrinsic initial guess also need to be
swapped.
```bash
ROS_NAMESPACE=/gt_box rosrun grand_tour_ceres_apps camera_camera_online_calibration -i /home/fu/catkin_ws/src/grand_tour_box/box_calibration/grand_tour_ceres_apps/config/initial_guess_hdr_front_right_swapped.yaml
```