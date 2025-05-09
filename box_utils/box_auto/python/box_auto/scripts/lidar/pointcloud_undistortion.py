#!/usr/bin/env python3

import os
import rosbag
from time import sleep
from box_auto.utils import get_bag, upload_bag, kill_roscore, WS
import tf
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage
import copy
from tqdm import tqdm
import numpy as np

PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"


def launch_undistorter(
    input_rosbag_path,
    input_trajectory_bag_path,
    input_tf_static_bag_path,
    output_bag_path,
    pcd_topic_in,
    child_frame,
    target_frame,
    unified_undistortion,
):
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    kill_roscore()

    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(1)
    command = (
        f"bash -c '"
        f"{PRE} "
        f"roslaunch pointcloud_undistortion undistort_pointcloud.launch "
        f"lidar_bag_path:={input_rosbag_path} "
        f"trajectory_bag_path:={input_trajectory_bag_path} "
        f"tf_static_bag_path:={input_tf_static_bag_path} "
        f"output_bag_path:={output_bag_path} "
        f"lidar_topic:={pcd_topic_in} "
        f"child_frame:={child_frame} "
        f"unified_undistortion:={unified_undistortion} "
        f"target_frame:={target_frame}'"
    )
    os.system(command)
    sleep(5)

    upload_bag(output_bag_path)


def switch_tf(exists_skip=False):
    if exists_skip:
        try:
            get_bag("*_lpc_tf_reverse.bag")
            return
        except:
            pass

    # Get input and output bag file names
    bag_in = get_bag("*_lpc_tf.bag")
    bag_out = bag_in.replace("_lpc_tf.bag", "_lpc_tf_reverse.bag")

    # Open input and output bag files
    with rosbag.Bag(bag_in, "r") as in_bag, rosbag.Bag(bag_out, "w", compression="lz4") as out_bag:
        tf_message_count = in_bag.get_message_count() - 1  # -1 to exclude the /tf_static message

        for topic, msg, t in tqdm(
            in_bag.read_messages(topics=["/tf"]), total=tf_message_count, desc="Processing TF messages"
        ):
            new_transforms = []
            for transform in msg.transforms:
                if transform.header.frame_id == "odom" and transform.child_frame_id == "base":
                    # Invert the transform
                    inverted_transform = TransformStamped()
                    inverted_transform.header = copy.deepcopy(transform.header)
                    inverted_transform.header.frame_id = transform.child_frame_id
                    inverted_transform.child_frame_id = transform.header.frame_id

                    # Extract the original rotation as a quaternion
                    original_quaternion = [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ]
                    # Compute the inverse rotation
                    inverted_quaternion = tf.transformations.quaternion_inverse(original_quaternion)

                    # Convert the inverted quaternion to a rotation matrix
                    R_inv = tf.transformations.quaternion_matrix(inverted_quaternion)[:3, :3]

                    # Extract the original translation vector
                    original_translation = np.array(
                        [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                        ]
                    )
                    # Compute the inverted translation: t_inv = -R_inv * t
                    inverted_translation = -np.dot(R_inv, original_translation)

                    inverted_transform.transform.translation.x = inverted_translation[0]
                    inverted_transform.transform.translation.y = inverted_translation[1]
                    inverted_transform.transform.translation.z = inverted_translation[2]

                    inverted_transform.transform.rotation = Quaternion(
                        x=inverted_quaternion[0],
                        y=inverted_quaternion[1],
                        z=inverted_quaternion[2],
                        w=inverted_quaternion[3],
                    )

                    new_transforms.append(inverted_transform)
                else:
                    new_transforms.append(transform)

            out_bag.write(topic, TFMessage(transforms=new_transforms), t)


if __name__ == "__main__":
    is_unified = True
    input_trajectory_bag_path = None
    print("Unified undistortion is enabled. Combined tf will be used.")
    input_trajectory_bag_path = get_bag("*tf_minimal.bag")

    # TF static
    input_tf_static_path = get_bag("*_tf_static_start_end.bag")

    # Remove existing undistorted bags if they exist
    try:
        existing_hesai_undist = get_bag("*_hesai_undist.bag")
        print(f"Removing existing hesai undistorted bag: {existing_hesai_undist}")
        os.remove(existing_hesai_undist)
    except:
        pass

    try:
        existing_livox_undist = get_bag("*_livox_undist.bag")
        print(f"Removing existing livox undistorted bag: {existing_livox_undist}")
        os.remove(existing_livox_undist)
    except:
        pass

    # Filtered Hesai bag
    input_hesai_bag_path = get_bag("*_nuc_hesai_ready.bag")
    output_hesai_bag_path = input_hesai_bag_path.replace("_nuc_hesai_ready.bag", "_nuc_hesai_undist.bag")
    launch_undistorter(
        input_hesai_bag_path,
        input_trajectory_bag_path,
        input_tf_static_path,
        output_hesai_bag_path,
        pcd_topic_in="/gt_box/hesai/points",
        child_frame="base",  # Used if not unified
        target_frame="odom",
        unified_undistortion=is_unified,
    )

    # Filtered Livox bag
    input_livox_bag_path = get_bag("*_nuc_livox_ready.bag")
    output_livox_bag_path = input_livox_bag_path.replace("_nuc_livox_ready.bag", "_nuc_livox_undist.bag")
    launch_undistorter(
        input_livox_bag_path,
        input_trajectory_bag_path,
        input_tf_static_path,
        output_livox_bag_path,
        pcd_topic_in="/gt_box/livox/lidar",
        child_frame="base",  # Used if not unified
        target_frame="odom",
        unified_undistortion=is_unified,
    )
    kill_roscore()
