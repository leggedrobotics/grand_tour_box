import yaml
import numpy as np
import pywavemap as wave
from tqdm import tqdm
from pathlib import Path

from tf_bag import BagTfTransformer
from rosbag import Bag
import tf
import ros_numpy
import rospy
from box_auto.utils import get_bag, WS, MISSION_DATA


def main():
    # Parameters
    date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob("*_nuc_livox*.bag")][0]
    OUTPUT = f"{date}_wavemap.wvmp"
    TF_BAG = get_bag("*_tf_static_dlio_tf.bag")
    POINTCLOUD_BAGS = [get_bag("*_dlio.bag"), get_bag("*_nuc_livox_filtered.bag")]
    OUTFILE = str(Path(POINTCLOUD_BAGS[0]).parent / OUTPUT)

    START_PLUS = 0
    END_MINUS = 0

    # Load config file
    with open(f"{WS}/src/grand_tour_box/box_bringup/bringup_wavemap/config/both.yaml", "r") as config_file:
        config = yaml.safe_load(config_file)

    # Create a map
    your_map = wave.Map.create(config["map"])

    # Create a measurement integration pipeline
    pipeline = wave.Pipeline(your_map)
    # Add map operations
    pipeline.add_operation({"type": "threshold_map", "once_every": {"seconds": 2.0}})

    # Add measurement integrators
    for name, cfg in config["measurement_integrators"].items():
        pipeline.add_integrator(name, cfg)

    # Integrate all the measurements
    tf_lookup = BagTfTransformer(TF_BAG)

    topic_to_integrator = {t["topic_name"]: t["measurement_integrator_names"] for t in config["inputs"]}

    for bag_path in POINTCLOUD_BAGS:
        with Bag(bag_path, "r") as bag:
            # Added tqdm for progress tracking (optional)
            total_messages = bag.get_message_count()
            start = bag.get_start_time() + START_PLUS
            end = bag.get_end_time() - END_MINUS
            count = 0
            for topic, msg, t in tqdm(
                bag.read_messages(
                    topics=[str(s) for s in topic_to_integrator.keys()],
                    start_time=rospy.Time(start),
                    end_time=rospy.Time(end),
                ),
                total=total_messages,
                desc=f"Processing {bag_path}",
            ):

                if topic in topic_to_integrator:  # PointCloud2 Message topics_to_numpy
                    integrator_name = topic_to_integrator[topic]
                    parent_frame = config["general"]["world_frame"]
                    child_frame = msg.header.frame_id
                    # Look up transform
                    try:
                        transform = tf_lookup.lookupTransform(parent_frame, child_frame, t)
                    except Exception as e:
                        print(f"Lookup Failed: {e}")
                        continue

                    rotation_matrix = tf.transformations.quaternion_matrix(transform[1])
                    se3_matrix = np.eye(4)
                    se3_matrix[:3, :3] = rotation_matrix[:3, :3]
                    se3_matrix[:3, 3] = transform[0]
                    pose = wave.Pose(se3_matrix)

                    # This is a slow and bad implementation
                    msg.fields = msg.fields[:5]
                    pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                    cloud = wave.Pointcloud(np.vstack((pc_data["x"], pc_data["y"], pc_data["z"])))

                    if type(integrator_name) is not list:
                        integ = [integrator_name]
                    else:
                        integ = integrator_name

                    pipeline.run_pipeline(integ, wave.PosedPointcloud(pose, cloud))
                    count += 1

                    print("count: ", count)

    # Remove map nodes that are no longer needed
    your_map.prune()

    # Save the map
    print(f"Saving map of size {your_map.memory_usage} bytes")
    your_map.store(OUTFILE)

    # Avoids leak warnings on old Python versions with lazy garbage collectors
    del pipeline, your_map

    # mkdir grand_tour_box/box_applications/wavemap/examples/cpp/build
    # cd grand_tour_box/box_applications/wavemap/examples/cpp/build; cmake ..; make
    # ./grand_tour_box/box_applications/wavemap/examples/cpp/build/conversions/occupancy_to_ply wavemap.wvmp


if __name__ == "__main__":
    main()
    exit(0)
