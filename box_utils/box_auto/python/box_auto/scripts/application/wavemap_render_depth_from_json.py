import argparse
import os
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import pywavemap as wm
import rosbag
import cv2
from box_auto.utils import BOX_AUTO_DIR, get_bag, get_file
import json
import time


class Saver:
    def __init__(self, folder):
        print("Saving to folder:", folder)
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.bag = None
        self.count = 0

    def save_png(self, depth_image, postfix, idx_image):
        output_path = self.folder / f"{postfix}_{idx_image}.png"
        invalid = depth_image == -1
        depth_image = depth_image.clip(0, (2**16 - 1) / 1000)
        depth_image = (depth_image * 1000).astype(np.uint16)  # Scale and convert to uint16
        depth_image[invalid] = 0
        Image.fromarray(depth_image.T).save(str(output_path))
        self.count += 1

    def __del__(self):
        if self.bag:
            self.bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract rectified depth maps from wavemap maps at given sensor poses."
    )
    parser.add_argument(
        "--map_file",
        type=str,
        default="/data/wavemap_scratch/wavemap/2024-10-01-11-29-55_wavemap.wvmp",
        help="Path to the wavemap file (.wvmp)",
    )
    parser.add_argument(
        "--json_file",
        type=str,
        default="/media/jonfrey/BoxiS4-2TB/deployment_day_1/2024-10-01-11-29-55/nerf_studio/transforms.json",
        help="Path to the nerfstudio file (.json)",
    )
    parser.add_argument(
        "--config_file",
        type=str,
        default=str(Path(BOX_AUTO_DIR).parent / "box_converter/nerfstudio/cfg/grand_tour_release.yaml"),
        help="Path to the configuration YAML file",
    )

    args = parser.parse_args()

    if args.map_file == "automatic":
        args.map_file, suc = get_file("*.wvmp", rglob=True)
        if not suc:
            exit(-1)

    # Validate map file
    if not os.path.exists(args.map_file):
        raise FileNotFoundError(f"Map file not found: {args.map_file}")

    # Load configuration
    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    # Load map
    your_map = wm.Map.load(args.map_file)

    # Load camera infos and create wavemap renderer
    renderer = {}
    for camera in config["cameras"]:
        bag_file = get_bag(camera["bag_pattern_info"])
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
                        "W": msg.width,
                        "H": msg.height,
                        "K": K_new,
                        "map1": map1,
                        "map2": map2,
                    }

                    print("Found camera:", camera["name"])
                    break

    # Load the JSON file
    with open(args.json_file, "r") as f:
        json_data = json.load(f)

    mission_name = args.map_file.split("/")[-1].split("_")[0]
    folder_path = Path("/data/depth_scratch") / mission_name / config["wavemap"]["key"]

    saver = Saver(folder_path)

    # Process cameras
    for j, frame in enumerate(json_data["frames"]):
        start = time.time()

        file_path = frame["file_path"]
        transform_matrix = np.array(frame["transform_matrix"])

        # Determine which camera the frame belongs to
        for camera in config["cameras"]:
            if camera["name"] in file_path:
                camera_name = camera["name"]
                break

        if camera_name is None:
            print(f"Warning: No matching camera found for frame {file_path}")
            continue

        # Extract rotation and translation from the transform matrix
        rot_matrix = transform_matrix[:3, :3]
        trans = transform_matrix[:3, 3]

        # Create the pose
        pose = wm.Pose(wm.Rotation(rot_matrix), np.array(trans))

        # Render the depth image
        undistorted_depth_image = renderer[camera_name]["renderer"].render(pose).data.astype(np.float64)
        saver.save_png(undistorted_depth_image, camera_name, idx_image=frame["file_path"][:-4].split("_")[-1])
        print(time.time() - start, "s   -   Iterations: ", j)

    exit(0)
