# Calibration Toolkits

### Scrutinize camera/LiDAR overlays
```bash
roslaunch box_calibration visualize_overlay_nuc.launch
```

Licensing: Most packages in this folder are licensed under the MIT License. 
The **grand_tour_camera_detectors** package includes third-party code developed by the 
**Autonomous Systems Lab and Skybotix AG** under a BSD-style license with an advertising clause. 
See that package's **LICENSE_ASL_SKYBOTIX.txt** for details.

## Batch calibration

Run `rosrun box_calibration process_all_calibrations.py --help` for the camera,
lidar, prism and optional camera–IMU recording folders. `--output-dir` must name
a new directory (default `calibration_output`). Reports and intermediate files
are written there; generated TF and camera YAMLs are staged under `deployment/`
for review. Input bags are read without reindexing or rewriting them. A failed
external command stops the workflow. `--dry_run` prints the plan without writing
files or invoking calibration programs.

The camera–IMU step can also run separately:

```sh
rosrun box_calibration process_camera_imu_calibrations.py /path/to/recording \
  --calibration /path/to/camera_calibration.yaml --output /path/to/imu_calibration.yaml
```

The output parent must exist. Input folders may contain existing camera detection
bags or decoded `sensor_msgs/Image` bags. The configured IMU topics come from
`calibration/allan_variances/*/imu.yaml`; unavailable topics are reported and
skipped. Use `--imu-topic ap20=/processed/topic` (also supported by the full
workflow) to select a prepared `sensor_msgs/Imu` stream explicitly. Nested AP20
debug messages are not converted to arrival-time IMU samples. At least one
configured IMU must be available. All selected solvers must succeed before the
aggregate YAML is replaced; temporary camera–IMU bags are removed on success or
failure. Camera–IMU time-offset estimation remains unsupported.

## Metadata bags

`store_tf_static.launch` accepts `validity_start` and `validity_end` in
`YYYY-MM-DD-HH-MM-SS` format. Empty values retain the earliest calibration date
and the current time, respectively. The exporter requires all ten camera YAMLs
and writes `/tf_static`, `/calibration_metadata` and camera calibration topics.
It atomically replaces `bag_file` after writing a complete bag and also publishes
`tf_static_metadata_<start>__<end>.bag`. An existing dated bag is protected from
overwrite. The shipped launch exports the single box-model publisher's snapshot.

The existing dataset consumer selects calibration by `validity_start`; it does
not enforce `validity_end`. Its `tf_static_metadata_new_*.bag` naming convention
and the existing TF conversion stage remain separate from this exporter.

The recording and workflow regression tests can be run from the repository root
in a sourced ROS workspace after building the camera detection messages:

```sh
PYTHONPATH="$PWD/box_calibration/box_calibration/src:$PYTHONPATH" python3 -m unittest discover \
  -s box_calibration/box_calibration/test -p test_followup_workflows.py -v
python3 -m unittest discover -s box_utils/box_recording/tests -v
```

The full-workflow test substitutes external calibration solves and runs the
actual graph conversion and metadata loading. The separate IMU/prism CLI tests
exercise the real solvers with independently generated observations.
