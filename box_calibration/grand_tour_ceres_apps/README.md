# Calibration applications

This package provides online/offline camera-to-camera calibration, camera–prism
calibration, and an offline Ceres camera–IMU solver. It requires ROS Noetic,
Ceres 2.2+, Eigen, OpenCV and yaml-cpp. The package uses C++17 for compatibility
with Noetic message serialization. CMake downloads the pinned Rerun 0.30 SDK;
Sophus headers and their license are included in `Sophus/`.

## Camera-to-camera

Input bags contain `grand_tour_camera_detection_msgs/CameraDetections` on
`<image_topic>_corner_detections`. The YAML topic/frame mapping must match the
messages. Initial parameters select the cameras to calibrate.

```sh
rosrun grand_tour_ceres_apps camera_camera_offline_calibration \
  --initial_guess initial.yaml --rostopic_frameid_mapping mapping.yaml \
  --bags detections.bag --output_path results/cameras.yaml
```

The output directory must already exist. Offline calibration checks that every
requested camera has observations connected to the fixed origin camera. It
solves, rebuilds accepted observations with the existing outlier filter, and
solves again. Input, convergence and write failures return a nonzero exit code.
The requested output path is honored. Existing deployed calibration files are
only changed when explicitly selected as output or installed by the existing
calibration deployment scripts.

Output includes intrinsics, distortion, transforms and, when covariance
estimation succeeds, `covariances.fxfycxcy_sigma` and `covariances.rtvec_sigma`.
These are standard deviations, not full covariance matrices. A missing
covariance means it could not be estimated; it does not mean zero uncertainty.
`residuals_path` refers to the per-camera observations and reprojection errors
written alongside the calibration. All generated files must be kept together.

## Camera–IMU

The reusable Aprilgrid detector preprocesses raw `sensor_msgs/Image` bags:

```sh
rosrun grand_tour_ceres_apps camera_imu_preprocess \
  --bags camera_images.bag --output detections.bag
rosrun grand_tour_ceres_apps camera_imu_offline_calibration \
  -c cameras.yaml --camera_detection_bags detections.bag \
  --imu_bag imu.bag --imu_topic /gt_box/stim320/imu \
  --output_path results/camera_imu.yaml
```

The preprocessor uses the existing Grand Tour 6 × 6 Aprilgrid with 0.083 m tags
and spacing ratio 0.3. It refuses existing output paths and removes its temporary
bag if conversion fails or no target is detected. Sensor timestamps are preserved.

The IMU path estimates camera-bundle/IMU extrinsics, gravity direction and
per-keyframe pose, velocity and biases. Its output stores the estimated biases
under nanosecond timestamp keys. Camera and IMU timestamps must already share a
clock; time-offset estimation is not implemented and that option is rejected.
The existing dataset-processing and calibration-deployment orchestration is
preserved; this solver is invoked explicitly through the command above.

## Visualization

For the camera-to-camera and camera–IMU executables, add `--visualize` to open the Rerun viewer, or
`--rerun_recording results/calibration.rrd` to save visualization without a GUI.
When both are supplied, saving takes precedence. `--blueprint_path` selects an
optional layout; absent layouts use Rerun's default. Opening a GUI requires a
compatible Rerun viewer on `PATH`. The blueprint generator scripts in
`grand_tour_calibration_viewers/scripts` require Python 3.10+ and
`rerun-sdk==0.30.0`; the ROS Noetic Python interpreter remains unchanged. Camera-to-camera views show
detections, coverage, camera connectivity, transforms, uncertainty and residuals.
Camera–IMU views show observations, trajectories, biases and residuals.

## Tests and validation limits

`catkin_make run_tests_grand_tour_ceres_apps` runs synthetic offline calibration,
input/output failure tests, analytical IMU residual checks, and utility tests.
Temporary bags and output files are removed by the tests. The full synthetic
camera–IMU executable and Rerun recording test can also be repeated in the sourced
workspace:

```sh
python3 tests/test_camera_imu_cli.py --executable /path/to/camera_imu_offline_calibration
```

This requires the ROS Python messages, NumPy, SciPy and PyYAML. It independently
generates motion and sensor observations, then checks convergence, rotation,
translation, per-keyframe bias output and recording-file creation.

The recording regression is optional. Set `GT_CALIBRATION_BAG_DIR` to the
reference recording directory containing the ten bags listed in
`tests/test_calibration_regression.cpp` and `output_camera_camera_calibration.yaml`.
An explicitly configured but missing recording fails. Checks cover every camera,
all distortion coefficients, intrinsics (2 pixels), translation (2 mm), and
rotation (0.001 rad). A skipped recording regression is not an accuracy pass.

Synthetic tests and ordinary dataset smoke tests do not establish physical
calibration accuracy or adequate motion excitation for the new camera–IMU
solver. Validate its results against an independently trusted calibration before
using them as deployed sensor parameters.
