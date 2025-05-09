Two MIssion and we would read the information
import zarr

root_directory = "/home/babalars/_work/grand_tour_box/box_utils/box_converter/huggingface-dataset-conversion/data"

zarr_file = zarr.open(root_directory, mode="r")

print()
depth images load
images

alphasense_cam1
    sequence_id: shape, nr_message
    timestamp: shape, nr_message
alphasense_cam2
    ...
    ..
..
hesai
    points: shape, nr_message
    sequence_id: shape, nr_message
    timestamp: shape, nr_message
livox