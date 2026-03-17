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
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev
```

### Ceres version 2.20
This app requires version 2.2 of the ceres-solver. The can be downloaded [here](http://ceres-solver.org/ceres-solver-2.2.0.tar.gz)
The installation instructions can be found [here](http://ceres-solver.org/installation.html).
These are summarized as follows:
```bash
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev

tar zxf ceres-solver-2.2.0.tar.gz # Currently this has been untarred on the home folder
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.2.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install
```

TLDR; The ceres-solver build directory is right now in the home directory under `/home/rsl/ceres-solver-2.2.0/ceres-bin`


### Catkin Build
```bash
catkin build grand_tour_ceres_apps
```

### Expected camera topics and frame IDs

The calibration node expects the following 10 cameras to be present. These are defined in
`config/initial_guess.yaml` (intrinsics + extrinsics initial guess) and
`config/rostopic_frameid_mappings.yaml` (topic → frame ID mapping).
**You are responsible for ensuring these topics are published before launching the calibration.**

| Camera | ROS topic | Frame ID | Sensor | Resolution | Distortion model |
|--------|-----------|----------|--------|------------|-----------------|
| cam0 | `/gt_box/alphasense_driver_node/cam1` | `cam1_sensor_frame` | Alphasense | 1440×1080 | equidistant |
| cam1 | `/gt_box/alphasense_driver_node/cam2` | `cam2_sensor_frame` | Alphasense | 1440×1080 | equidistant |
| cam2 | `/gt_box/alphasense_driver_node/cam3` | `cam3_sensor_frame` | Alphasense | 1440×1080 | equidistant |
| cam3 | `/gt_box/alphasense_driver_node/cam4` | `cam4_sensor_frame` | Alphasense | 1440×1080 | equidistant |
| cam4 | `/gt_box/alphasense_driver_node/cam5` | `cam5_sensor_frame` | Alphasense | 1440×1080 | equidistant |
| cam5 | `/gt_box/hdr_front/image_raw` | `hdr_front` | HDR front | 1920×1280 | equidistant |
| cam6 | `/gt_box/hdr_left/image_raw` | `hdr_left` | HDR left | 1920×1280 | equidistant |
| cam7 | `/gt_box/hdr_right/image_raw` | `hdr_right` | HDR right | 1920×1280 | equidistant |
| cam8 | `/gt_box/zed2i_driver_node/left_raw/image_raw_color` | `zed2i_left_camera_optical_frame` | ZED2i left | 1920×1280 | radtan |
| cam9 | `/gt_box/zed2i_driver_node/right_raw/image_raw_color` | `zed2i_right_camera_optical_frame` | ZED2i right | 1920×1280 | radtan |

The calibration node subscribes to `<topic>_corner_detections` (i.e. the detection topic derived from each image topic above),
not the raw image topics directly. The detector nodes must be running and publishing on those derived topics.

To use a different camera set, update both config files and re-run.

### Running the camera camera calibration
First start up the detectors and the viewers. Note that the detectors are to be separately called from
the NUC and the Jetson, respectively. The viewer and the calibration node are run from the OPC.
```bash
roslaunch grand_tour_camera_detectors detectors_on_nuc_generic.launch use_april_grid:=true [record_pointcloud:=true (default: false)]
roslaunch grand_tour_camera_detectors detectors_on_jetson_generic.launch use_april_grid:=true
roslaunch grand_tour_calibration_viewers all_cameras.launch

ROS_NAMESPACE=/gt_box rosrun grand_tour_ceres_apps camera_camera_online_calibration 
```
Running the node above in the `/gt_box` namespace is required for smoothly interfacing with the detector and viewer nodes
if  the detectors are run with `record_pointcloud:=true`, the hesai and livox point clouds will also be recorded on the
NUC.


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