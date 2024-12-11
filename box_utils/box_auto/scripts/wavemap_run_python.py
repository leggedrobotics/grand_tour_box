import yaml  # Added yaml import
import numpy as np
import pywavemap as wave
from tf_bag import BagTfTransformer
from rosbag import Bag  # Added rosbag import
import tf
from tqdm import tqdm
import ros_numpy
from pytictac import CpuTimer

# Parameters
OUTPUT = "wavemap.wvmp"
TF_BAG = "/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/2024-10-01-11-29-55_tf_all.bag"
POINTCLOUD_BAGS = [
    "/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/2024-10-01-11-29-55_nuc_hesai_post_processed_undistorted.bag"
]
REFERENCE_FRAME = "enu_origin"

# Create a map
your_map = wave.Map.create({"type": "hashed_chunked_wavelet_octree", "min_cell_width": {"meters": 0.05}})

# Create a measurement integration pipeline
pipeline = wave.Pipeline(your_map)
# Add map operations
pipeline.add_operation({"type": "threshold_map", "once_every": {"seconds": 5.0}})

# Fixed yaml loading
with open("/home/jonfrey/git/grand_tour_box/box_bringup/bringup_wavemap/config/both.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

# Add measurement integrators
for name, cfg in config["measurement_integrators"].items():
    pipeline.add_integrator(name, cfg)

# Integrate all the measurements
current_index = 0
tf_lookup = BagTfTransformer(TF_BAG)

topic_to_integrator = {t["topic_name"]: t["measurement_integrator_names"] for t in config["inputs"]}

count = 0
for bag_path in POINTCLOUD_BAGS:
    with Bag(bag_path, "r") as bag:
        # Added tqdm for progress tracking (optional)
        total_messages = bag.get_message_count()

        for topic, msg, t in tqdm(
            bag.read_messages(topics=[str(s) for s in topic_to_integrator.keys()]),
            total=total_messages,
            desc=f"Processing {bag_path}",
        ):

            if topic in topic_to_integrator:  # PointCloud2 Message topics_to_numpy
                with CpuTimer("prepare data"):
                    integrator_name = topic_to_integrator[topic]

                    # Assuming you want to look up transform between parent and child frames
                    parent_frame = REFERENCE_FRAME  # Use REFERENCE_FRAME or specify correctly
                    child_frame = msg.header.frame_id  # Adjust based on your message type

                    # Look up transform
                    transform = tf_lookup.lookupTransform(parent_frame, child_frame, t)
                    rotation_matrix = tf.transformations.quaternion_matrix(transform[1])
                    se3_matrix = np.eye(4)
                    se3_matrix[:3, :3] = rotation_matrix[:3, :3]
                    se3_matrix[:3, 3] = transform[0]
                    pose = wave.Pose(se3_matrix)

                    # This is a slow and bad implementation

                    pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                    cloud = wave.Pointcloud(np.vstack((pc_data["x"], pc_data["y"], pc_data["z"])))

                with CpuTimer("integrate"):
                    pipeline.run_pipeline([integrator_name], wave.PosedPointcloud(pose, cloud))
                count += 1


# Remove map nodes that are no longer needed
your_map.prune()

# Save the map
print(f"Saving map of size {your_map.memory_usage} bytes")
your_map.store(OUTPUT)

# Avoids leak warnings on old Python versions with lazy garbage collectors
del pipeline, your_map
# mkdir grand_tour_box/box_applications/wavemap/examples/cpp/build
# cd grand_tour_box/box_applications/wavemap/examples/cpp/build; cmake ..; make
# ./grand_tour_box/box_applications/wavemap/examples/cpp/build/conversions/occupancy_to_ply wavemap.wvmp
