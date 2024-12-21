import os
import yaml
import argparse
from pathlib import Path
import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import rosbag
from cv_bridge import CvBridge
from tf_bag import BagTfTransformer
from sensor_msgs.msg import CameraInfo, CompressedImage

from torchvision.io.image import read_image
from torchvision.models.segmentation import fcn_resnet50, FCN_ResNet50_Weights

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")


def get_bag(directory, pattern):
    # Get reference bag path
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein download --mission {uuid} --dest {MISSION_DATA} '{pattern}'")

    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False
    return files[0], True


def undistort_image(image, camera_info, new_camera_info=None):
    K = np.array(camera_info.K).reshape((3, 3))
    D = np.array(camera_info.D)

    h, w = image.shape[:2]

    if new_camera_info is None:
        new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (w, h), np.eye(3), balance=0.0, fov_scale=1.0
        )

        new_camera_info = CameraInfo()
        new_camera_info.header = camera_info.header
        new_camera_info.width = camera_info.width
        new_camera_info.height = camera_info.height
        new_camera_info.K = new_camera_matrix.flatten().tolist()
        new_camera_info.D = [0, 0, 0, 0]  # Fisheye distortion coefficients
        new_camera_info.R = camera_info.R
        new_P = np.eye(4)
        new_P[:3, :3] = new_camera_matrix
        new_camera_info.P = new_P.flatten().tolist()
    else:
        new_camera_info.header = camera_info.header
        new_camera_matrix = np.array(new_camera_info.K).reshape((3, 3))

    # Initialize undistortion map
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2)

    # Apply undistortion
    undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_image, new_camera_info


class ImageSaver:
    def __init__(self, camera_infos, camera_keys, tf_listener, config, mission_data):
        self.tf_listener = tf_listener
        self.camera_infos = camera_infos
        self.camera_keys = camera_keys
        self.config = config
        self.blur_threshold = config["nerfstudio"]["blur_threshold"]

        self.taget_camera_infos = {k: None for k in camera_infos.keys()}

        self.output_folder = Path(mission_data) / "nerf_studio"
        self.output_folder.mkdir(parents=True, exist_ok=True)

        self.frames_json_file = self.output_folder / "transforms.json"
        self.images_folder = self.output_folder / "rgb"
        self.images_folder.mkdir(parents=True, exist_ok=True)

        self.frame_data = {"camera_model": "OPENCV", "frames": []}
        self.image_counters = {key: 0 for key in camera_keys.keys()}
        self.image_last_stored = {key: 0 for key in camera_keys.keys()}
        self.bridge = CvBridge()

        if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
            # Step 1: Initialize model with the best available weights
            self.weights = FCN_ResNet50_Weights.DEFAULT
            self.model = fcn_resnet50(weights=self.weights)
            self.model.eval()
            self.preprocess = self.weights.transforms()

            self.mask_folder = self.output_folder / "mask"
            self.mask_folder.mkdir(parents=True, exist_ok=True)

    def ros_to_gl_transform(self, transform_ros):
        cv_to_gl = np.eye(4)
        cv_to_gl[1:3, 1:3] = np.array([[-1, 0], [0, -1]])
        transform_gl = cv_to_gl @ transform_ros @ np.linalg.inv(cv_to_gl)
        return transform_gl

    def store_frame(self, img_msg, topic):

        if self.image_counters[topic] > self.config["num_images_per_topic"]:
            return False

        if img_msg.header.stamp.to_sec() - self.image_last_stored[topic] < (1 / self.config["hz"]) - 0.001:
            return True
        self.image_counters[topic] += 1
        self.image_last_stored[topic] = img_msg.header.stamp.to_sec()
        camera_key = self.camera_keys[topic]
        image_filename = f"{camera_key}_{img_msg.header.seq:05d}.png"
        image_path = self.images_folder / image_filename

        # Convert and save image
        # Convert image
        if isinstance(img_msg, CompressedImage) or "compressed" in topic:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
            # im_tensor = torch.from_numpy(cv_image).to(dtype=torch.float32) / 255.0
            # im_tensor = torch.flip(im_tensor, [-1])

        else:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
            # im_tensor = torch.from_numpy(cv_image).to(dtype=torch.float32) / 255.0

        # Undistort image if needed
        if "_rect" not in topic:
            cv_image, self.taget_camera_infos[topic] = undistort_image(
                cv_image, self.camera_infos[topic], self.taget_camera_infos[topic]
            )
            camera_info = self.taget_camera_infos[topic]
        else:
            camera_info = self.camera_infos[topic]
            camera_info.D = [0, 0, 0, 0]

        # Get transformation
        trans, quat_xyzw = self.tf_listener.lookupTransform(
            self.config["base_frame"], img_msg.header.frame_id, img_msg.header.stamp
        )
        if trans is None or quat_xyzw is None:
            print(f"Warning: Could not get transform for {camera_key} at time {img_msg.header.stamp}")
            return True

        # Check blur and save image
        blur = cv2.Laplacian(cv_image, cv2.CV_64F).var()
        if blur < self.blur_threshold:
            print(f"Warning: Image too blurry (blur value: {blur}). Skipping.")
            return True

        cv2.imwrite(str(image_path), cv_image)

        if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
            # Pretty bad implementaion
            img = read_image(str(image_path))
            batch = self.preprocess(img).unsqueeze(0)
            prediction = self.model(batch)["out"]
            normalized_masks = prediction.softmax(dim=1)
            class_to_idx = {cls: idx for (idx, cls) in enumerate(self.weights.meta["categories"])}
            mask = normalized_masks[0, class_to_idx["person"]]
            humans = mask > 0.5
            output = humans.cpu().numpy()

            # Resize the output mask to the original image size
            _, original_height, original_width = img.shape
            resized_output = cv2.resize(output.astype(np.float32), (original_width, original_height))

            # Invert the mask: 1 where there is no human, 0 where there is a human
            inverted_mask = 1 - resized_output

            # Scale the mask to 0-255 range and convert to uint8
            mask_image = (inverted_mask * 255).astype(np.uint8)

            # Save the mask image
            cv2.imwrite(str(image_path).replace("/rgb/", "/mask/"), mask_image)

        # Create transformation matrix using scipy Rotation
        try:
            rotation = R.from_quat(quat_xyzw).as_dcm()
        except:
            rotation = R.from_quat(quat_xyzw).as_matrix()
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation
        transform_matrix[:3, 3] = trans

        # Convert to OpenGL convention
        transform_matrix = self.ros_to_gl_transform(transform_matrix)

        # Add frame data
        frame_data = {
            "file_path": f"./rgb/{image_filename}",
            "transform_matrix": transform_matrix.tolist(),
            "fl_x": str(camera_info.K[0]),
            "fl_y": str(camera_info.K[4]),
            "cx": str(camera_info.K[2]),
            "cy": str(camera_info.K[5]),
            "w": str(camera_info.width),
            "h": str(camera_info.height),
            "k1": str(camera_info.D[0]),
            "k2": str(camera_info.D[1]),
            "p1": str(camera_info.D[2]),
            "p2": str(camera_info.D[3]),
        }
        if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
            frame_data["mask_path"] = f"./mask/{image_filename}"
        self.frame_data["frames"].append(frame_data)
        return True

    def save_json(self):
        with open(self.frames_json_file, "w") as f:
            json.dump(self.frame_data, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Process ROS bags and extract camera images and transformations.")

    PATH = "/home/jonfrey/git/grand_tour_box/box_utils/box_converter/grand_tour_offline.yaml"
    parser.add_argument("--mission_data", type=str, default=MISSION_DATA, help="Mission Folder")
    parser.add_argument("--config_file", type=str, default=PATH, help="Path to the configuration YAML file")
    args = parser.parse_args()

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    input_folder = args.mission_data
    tf_bag_path, suc = get_bag(args.mission_data, "*_tf_static_dlio_tf.bag")

    # Load camera infos
    camera_infos = {}
    camera_keys = {}
    for camera in config["cameras"]:
        bag_file, suc = get_bag(args.mission_data, camera["bag_pattern"])
        if not suc:
            print("Failed to find: ", camera["bag_pattern"])

        with rosbag.Bag(bag_file, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=[camera["info_topic"]]):
                camera_infos[camera["image_topic"]] = msg
                camera_keys[camera["image_topic"]] = camera["name"]
                break

    # Initialize TF listener
    tf_listener = BagTfTransformer(tf_bag_path)

    # Initialize ImageSaver
    image_saver = ImageSaver(camera_infos, camera_keys, tf_listener, config, args.mission_data)

    # Process image messages
    for camera in config["cameras"]:
        bag_file = next(Path(input_folder).glob(camera["bag_pattern"]))
        with rosbag.Bag(bag_file, "r") as bag:
            start = bag.get_start_time() + config["skip_start_seconds"]
            end = bag.get_end_time() - config["skip_end_seconds"]
            for topic, msg, t in bag.read_messages(
                topics=[camera["image_topic"]], start_time=rospy.Time(start), end_time=rospy.Time(end)
            ):
                suc = image_saver.store_frame(msg, topic)
                if not suc:
                    print(f"Finished processing {camera['name']}")
                    break

    # Save JSON file
    image_saver.save_json()


if __name__ == "__main__":
    main()
