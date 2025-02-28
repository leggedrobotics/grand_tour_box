import argparse
import os
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import pywavemap as wm
import rosbag
import rospy
from tf_bag import BagTfTransformer
from sensor_msgs.msg import Image as ImageRos
import tf.transformations
import matplotlib.pyplot as plt
import cv2
from box_auto.utils import MISSION_DATA, BOX_AUTO_DIR, get_bag, upload_bag, get_file


class Saver:
    def __init__(self, folder, save_rosbag=True, rosbag_prefix=""):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.bag = None

        if save_rosbag:
            self.bag_path = self.folder / f"{rosbag_prefix}_wavemap.bag"
            self.bag = rosbag.Bag(self.bag_path, "w", compression="lz4")

    def save_ros(self, depth_image, topic, header=None, t=None):
        msg = ImageRos()
        msg.header = header
        msg.height, msg.width = depth_image.T.shape
        msg.encoding = "32FC1"  # Float32
        msg.is_bigendian = False
        msg.step = depth_image.T.shape[1] * 4  # 4 bytes per float
        msg.data = depth_image.T.astype(np.float32).tobytes()

        if t is None:
            t = msg.header.stamp
        self.bag.write(topic, msg, t)

    def save_png(self, depth_image, postfix, header=None):
        output_path = self.folder / f"{postfix}_{header.seq:05d}.png"
        invalid = depth_image == -1
        depth_image = depth_image.clip(0, (2**16 - 1) / 1000)
        depth_image = (depth_image * 1000).astype(np.uint16)  # Scale and convert to uint16
        depth_image[invalid] = 0
        Image.fromarray(depth_image.T).save(str(output_path))

    def __del__(self):
        if self.bag:
            self.bag.close()


def plot_depth_image(depth_image, max_range, colormap="viridis", figsize=(10, 8)):
    """
    Plot a depth image with Viridis colormap.

    Parameters:
    -----------
    depth_image : numpy.ndarray - dtype float32
        2D array containing depth values W,H - values in m
    max_range : float, optional
        Maximum depth range for color scaling.
        If None, uses the maximum value in the image.
    colormap : str, optional
        Matplotlib colormap name. Defaults to 'viridis'.
    figsize : tuple, optional
        Figure size (width, height). Defaults to (10, 8).

    Returns:
    --------
    matplotlib.figure.Figure
        Figure object for further customization if needed
    """
    width, height = depth_image.shape

    # Create the figure with calculated dimensions
    max_dimension = 10
    aspect_ratio = width / height
    if aspect_ratio > 1:
        fig_width = min(max_dimension, max_dimension)
        fig_height = fig_width / aspect_ratio
    else:
        fig_height = min(max_dimension, max_dimension)
        fig_width = fig_height * aspect_ratio
    plt.figure(figsize=(fig_width, fig_height))

    # Create the plot with specified colormap
    im = plt.imshow(depth_image.T, cmap=colormap, vmin=0, vmax=max_range)

    # Add colorbar
    plt.colorbar(im, label="Depth")

    # Remove axis ticks
    plt.xticks([])
    plt.yticks([])

    # Title
    plt.title("Depth Image")

    # Show the plot
    plt.tight_layout()
    plt.show()

    return plt.gcf()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract rectified depth maps from wavemap maps at given sensor poses."
    )
    parser.add_argument(
        "--map_file",
        type=str,
        default="automatic",
        help="Path to the wavemap file (.wvmp)",
    )
    parser.add_argument("--mission_data", type=str, default=MISSION_DATA, help="Path to the mission data folder")
    parser.add_argument(
        "--config_file",
        type=str,
        default=str(Path(BOX_AUTO_DIR).parent / "box_converter/grand_tour_offline.yaml"),
        help="Path to the configuration YAML file",
    )

    args = parser.parse_args()

    if args.map_file == "automatic":
        args.map_file, suc = get_file("*.wvmp")
        if not suc:
            exit(-1)

    # Validate map file
    if not os.path.exists(args.map_file):
        raise FileNotFoundError(f"Map file not found: {args.map_file}")

    # Load configuration
    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    # Retrieve tf bag
    tf_bag_path = get_bag("*_tf_static_hesai_dlio.bag")
    prefix = Path(tf_bag_path).stem.split("_")[0]

    tf_listener = BagTfTransformer(tf_bag_path)

    # Load map
    your_map = wm.Map.load(args.map_file)

    # Load camera infos
    renderer = {}
    for camera in config["cameras"]:
        bag_file = get_bag(args.mission_data, camera["bag_pattern"])

        with rosbag.Bag(bag_file, "r") as bag:
            for _topic, msg, _t in bag.read_messages(topics=[camera["info_topic"]]):
                if camera["name"] not in renderer:
                    # TODO add implementation for no distortion -> Check distortion params
                    if msg.distortion_model == "equidistant":
                        # Create the depth image renderer
                        K = np.array(msg.K).reshape((3, 3))
                        D = np.array(msg.D)

                        K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                            K, D, (msg.width, msg.height), np.eye(3), balance=0.0, fov_scale=1.0
                        )
                        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                            K, D, np.eye(3), K_new, (msg.width, msg.height), cv2.CV_16SC2
                        )
                    else:
                        raise ValueError("Not implemented")

                    renderer[camera["name"]] = {
                        "renderer": wm.RaycastingRenderer(
                            your_map,
                            wm.Projector.create(
                                {
                                    "type": "pinhole_camera_projector",
                                    "width": msg.width,
                                    "height": msg.height,
                                    "fx": K_new[0, 0],
                                    "fy": K_new[1, 1],
                                    "cx": K_new[0, 2],
                                    "cy": K_new[1, 2],
                                }
                            ),
                            config["wavemap"]["log_odds_occupancy_threshold"],
                            config["wavemap"]["max_range"],
                            config["wavemap"]["default_depth_value"],
                        ),
                        "map1": map1,
                        "map2": map2,
                    }
                    break

    saver = Saver(
        Path(args.mission_data) / "wavemap", save_rosbag=config["wavemap"]["save_ros"], rosbag_prefix="prefix"
    )

    # Process cameras
    for camera in config["cameras"]:
        bag_file = get_bag(camera["bag_pattern"])

        with rosbag.Bag(bag_file, "r") as bag:

            start_time = bag.get_start_time() + config["skip_start_seconds"]
            end_time = bag.get_end_time() - config["skip_end_seconds"]
            image_last_stored = {}
            count = {}
            for topic, msg, t in bag.read_messages(
                topics=[camera["image_topic"]],
                start_time=rospy.Time(start_time),
                end_time=rospy.Time(end_time),
            ):
                if count.get(topic, 0) > config["num_images_per_topic"]:
                    continue

                if msg.header.stamp.to_sec() - image_last_stored.get(topic, 0) < (1 / config["hz"]) - 0.001:
                    continue

                count[topic] = count.get(topic, 0) + 1
                image_last_stored[topic] = msg.header.stamp.to_sec()

                try:
                    trans, quat = tf_listener.lookupTransform(
                        config["base_frame"], msg.header.frame_id, msg.header.stamp
                    )
                except Exception as e:
                    print(f"Transform lookup failed: {e}")
                    continue

                rot_so3 = tf.transformations.quaternion_matrix(quat)[:3, :3]

                pose = wm.Pose(wm.Rotation(rot_so3), np.array(trans))
                undistorted_depth_image = renderer[camera["name"]]["renderer"].render(pose).data

                # plot_depth_image(undistorted_depth_image, max_range=config["wavemap"]["max_range"], colormap='viridis')

                if config["wavemap"]["save_ros"]:
                    saver.save_ros(undistorted_depth_image, topic, msg.header, t)
                if config["wavemap"]["save_png"]:
                    saver.save_png(undistorted_depth_image, camera["name"], msg.header)

    print("Processing complete.")

    if config["wavemap"]["save_ros"]:
        upload_bag(saver.bag_path)
    exit(0)
