## Creating the Protobuf Files
```
cd /home/jonfrey/git/grand_tour_box/box_utils/box_recording; python -m grpc_tools.protoc -I=proto --python_out=./scripts --grpc_python_out=./scripts proto/start_recording.proto
```
```
cd /home/jonfrey/git/grand_tour_box/box_utils/box_recording; python -m grpc_tools.protoc -I=proto --python_out=./scripts --grpc_python_out=./scripts proto/stop_recording.proto
```
```
cd /home/jonfrey/git/grand_tour_box/box_utils/box_recording; python -m grpc_tools.protoc -I=proto --python_out=./scripts --grpc_python_out=./scripts proto/recording_status.proto
```


## Development
python3 /home/jonfrey/git/grand_tour_box/box_utils/box_recording/scripts/rosbag_record_coordinator.py

python3 /home/jonfrey/git/grand_tour_box/box_utils/box_recording/scripts/rosbag_record_node_grpc.py

rosservice call /rosbag_record_robot_coordinator/stop_recording "verbose: false"   

rosservice call /rosbag_record_robot_coordinator/start_recording "yaml_file: 'debug'"