## Creating the Protobuf Files
```
cd /home/jonfrey/git/grand_tour_box/box_utils/box_recording; python -m grpc_tools.protoc -I=proto --python_out=./scripts --grpc_python_out=./scripts proto/start_recording.proto
```
```
cd /home/jonfrey/git/grand_tour_box/box_utils/box_recording; python -m grpc_tools.protoc -I=proto --python_out=./scripts --grpc_python_out=./scripts proto/stop_recording.proto
```


## Development
python3 /home/jonfrey/git/grand_tour_box/box_utils/box_recording/scripts/rosbag_record_coordinator.py

python3 /home/jonfrey/git/grand_tour_box/box_utils/box_recording/scripts/rosbag_record_node_grpc.py

rosservice call /rosbag_record_robot_coordinator/stop_recording "verbose: false"   

rosservice call /rosbag_record_robot_coordinator/start_recording "yaml_file: 'debug'"

## Recording destinations and CPT7 diagnostics

The nodes create the selected `~data_path` when necessary and fail startup if it
cannot be created. Defaults remain `/data` for the normal recorder and the
package's `data` directory for the gRPC recorder. Debug-log collection respects
`ROS_LOG_DIR`, then `ROS_HOME`, then the current user's `~/.ros` directory, using
the existing recording time window.

`~cpt7_record_diagnostics` defaults to `true` and enables ten extra ALIGN,
dual-antenna, TerraStar, tracking and PPP/PSR log requests after the existing
Inertial Explorer requests. Unsupported diagnostics produce warnings. Required
log failures still make startup report failure, while later required requests
are attempted. Receiver firmware acceptance must be checked on the CPT7 hardware.
