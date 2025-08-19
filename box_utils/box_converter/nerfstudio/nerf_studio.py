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
import os
import kleinkram
import torch
from transformers import Mask2FormerImageProcessor, Mask2FormerForUniversalSegmentation
from PIL import Image
import sensor_msgs.point_cloud2 as pc2


# Required for visualization of depth
from scipy.ndimage import grey_dilation
import matplotlib.pyplot as plt

try:
    from box_auto.utils import MISSION_DATA, WS, ARTIFACT_FOLDER
    from box_auto.utils import get_uuid_mapping, get_bag

    USE_KLEINKRAM = True

except:
    print("Failed to import box_auto.utils, using local paths")
    print("Please ensure that the you have the files correctly downloaded")
    MISSION_DATA = Path("/data")
    WS = Path(__file__).parent.parent.parent.parent.parent
    ARTIFACT_FOLDER = Path("/out")
    USE_KLEINKRAM = False

    def get_bag(pattern, directory, try_until_suc=False):
        """
        Get the first bag file matching the pattern in the specified directory.
        If try_until_suc is True, it will keep trying until a bag file is found.
        """
        paths = [path for path in Path(directory).rglob(pattern)]
        if len(paths) == 1:
            return str(paths[0])
        else:
            print(f"Found {len(paths)} bag files matching {pattern} in {directory}")
            raise FileNotFoundError(f"No bag file found matching {pattern} in {directory}. Found: {paths}")


def undistort_image(image, camera_info, new_camera_info, map1, map2, invalid_mask, fisheye=True):
    K = np.array(camera_info.K).reshape((3, 3))
    D = np.array(camera_info.D)
    h, w = image.shape[:2]

    if new_camera_info is None:
        if fisheye:
            new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=1.0, fov_scale=1.0
            )
            D_new = [0, 0, 0, 0]
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2)

        else:
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
            D_new = [0, 0, 0, 0, 0]
            map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

        new_camera_info = CameraInfo()
        new_camera_info.header = camera_info.header
        new_camera_info.width = camera_info.width
        new_camera_info.height = camera_info.height
        new_camera_info.K = new_camera_matrix.flatten().tolist()
        new_camera_info.D = D_new
        new_camera_info.R = camera_info.R
        new_P = np.eye(4)
        new_P[:3, :3] = new_camera_matrix
        new_camera_info.P = new_P.flatten().tolist()
        valid_mask = cv2.remap(
            np.ones(image.shape[:2], dtype=np.uint8),
            map1,
            map2,
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
        )
        invalid_mask = valid_mask == 0

    else:
        new_camera_info.header = camera_info.header

    if not fisheye:
        # HACKY way to deal with wrong camera info of zed2i
        image = cv2.flip(image, 0)
        undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        undistorted_image = cv2.flip(undistorted_image, 0)
    else:
        undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_image, new_camera_info, map1, map2, invalid_mask


def ros_to_gl_transform(transform_ros):
    cv_to_gl = np.eye(4)
    cv_to_gl[1:3, 1:3] = np.array([[-1, 0], [0, -1]])
    transform_gl = cv_to_gl @ transform_ros @ np.linalg.inv(cv_to_gl)
    return transform_gl


def gl_to_ros_transform(transform_gl):
    cv_to_gl = np.eye(4)
    cv_to_gl[1:3, 1:3] = np.array([[-1, 0], [0, -1]])
    transform_ros = np.linalg.inv(cv_to_gl) @ transform_gl @ cv_to_gl
    return transform_ros


class DepthProcessor:
    def __init__(self, config, tf_listener, output_folder, bag_folder):
        self.debug = config["nerfstudio"]["debug"]
        self.config = config
        self.sub_key = "depth"

        self.lidar_msgs = {}
        self.lidar_accumulate_scans = {}
        for lidar_cfg in config["nerfstudio"]["depth_processor"]["lidars"]:
            lidar_bag_path = get_bag(lidar_cfg["bag_pattern_lidar"], directory=bag_folder, try_until_suc=False)
            self.lidar_msgs[lidar_cfg["lidar_topic"]] = []
            self.lidar_accumulate_scans[lidar_cfg["lidar_topic"]] = lidar_cfg["accumulate_scans"]

            with rosbag.Bag(lidar_bag_path) as lidar_bag:
                for _, msg, _ in lidar_bag.read_messages(topics=[lidar_cfg["lidar_topic"]]):
                    self.lidar_msgs[lidar_cfg["lidar_topic"]].append(msg)

        self.tf_listener = tf_listener
        self.depth_folder = output_folder / self.sub_key
        self.depth_folder.mkdir(parents=True, exist_ok=True)
        self.output_folder = output_folder

    def transform_lidar_msgs_to_single_frame(self, closest_lidar_msgs):
        # TODO
        msg_0 = closest_lidar_msgs[0]
        # Get transform from LiDAR to world
        msg0_to_world = self.tf_listener.lookupTransform(
            self.config["tf_config"]["world_frame"],
            msg_0.header.frame_id,
            time=msg_0.header.stamp,
            method=self.config["tf_config"]["tf_lookup_method"],
            odom_world=self.config["tf_config"]["odom_world"],
            odom_body=self.config["tf_config"]["odom_body"],
            return_se3=True,
        )
        world_to_msg0 = np.linalg.inv(msg0_to_world)

        lidar_points = self.pointcloud2_to_xyz(msg_0)

        def transform_points(points, transform_matrix):
            points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
            return (transform_matrix @ points_homo.T).T[:, :3]

        if len(closest_lidar_msgs) > 1:
            # If there are multiple LiDAR messages, accumulate points
            for msg in closest_lidar_msgs[1:]:
                lidar_to_world_msg = self.tf_listener.lookupTransform(
                    self.config["tf_config"]["world_frame"],
                    msg.header.frame_id,
                    time=msg.header.stamp,
                    method=self.config["tf_config"]["tf_lookup_method"],
                    odom_world=self.config["tf_config"]["odom_world"],
                    odom_body=self.config["tf_config"]["odom_body"],
                    return_se3=True,
                )
                lidar_points_msg = self.pointcloud2_to_xyz(msg)

                # Transform points to world frame and the to msg0 frame
                lidar_points_world = transform_points(lidar_points_msg, lidar_to_world_msg)
                lidar_points_msg_in_msg0_frame = transform_points(lidar_points_world, world_to_msg0)
                lidar_points = np.vstack((lidar_points, lidar_points_msg_in_msg0_frame))

        return lidar_points, msg0_to_world

    def process(self, frame, debug_tag):
        camera_to_world = gl_to_ros_transform(np.array(frame["transform_matrix"]))
        tim = rospy.Time()
        tim.secs = int(frame["timestamp"].split("_")[0])
        tim.nsecs = int(frame["timestamp"].split("_")[1])

        closest_lidar_msgs = self.find_closest_lidar_msgs(tim)

        if closest_lidar_msgs is None or len(closest_lidar_msgs) == 0:
            print("No LiDAR data found")
            return False

        lidar_points, lidar_to_world = self.transform_lidar_msgs_to_single_frame(closest_lidar_msgs)

        if len(lidar_points) == 0:
            print("No valid points in LiDAR message for frame")
            return False

        # Camera to world transform (from nerfstudio JSON)
        world_to_camera = np.linalg.inv(camera_to_world)
        lidar_to_camera = world_to_camera @ lidar_to_world
        # Convert PointCloud2 to numpy array

        camera_intrinsics = self.get_camera_intrinsics(frame)

        # TODO: Very slow implementation
        # Project LiDAR points onto camera image
        depth_image = self.project_lidar_to_camera(
            lidar_points, camera_intrinsics, lidar_to_camera, int(frame["w"]), int(frame["h"])
        )

        invalid = depth_image == -1

        depth_image = depth_image.clip(0, 300)  # Clip to a reasonable max depth in meters
        depth_image = depth_image.astype(np.float32)
        depth_image[invalid] = 0.0

        # Save depth image
        frame["depth_file_path"] = frame["file_path"].replace("rgb", self.sub_key).replace(".jpg", ".png")

        cv2.imwrite(str(self.output_folder / frame["depth_file_path"]), depth_image.astype(np.float32))

        if self.debug:
            self.overlay_depth_on_rgb(frame, debug_tag)

        return frame

    def overlay_depth_on_rgb(self, frame, debug_tag):
        rgb_image = np.array(Image.open(self.output_folder / frame["file_path"]))
        depth_image = cv2.imread(str(self.output_folder / frame["depth_file_path"]), cv2.IMREAD_UNCHANGED)
        # Normalize depth image for visualization - max range 10m
        depth_normalized = (depth_image.clip(0, 10.0) / 10.0 * 255).astype(np.uint8)

        # Dilate the depth image to increase pixel width to 3
        depth_normalized = grey_dilation(depth_normalized, size=(3, 3))

        # rgb_image H,W,3
        alpha = 0

        cmap = plt.get_cmap("turbo").reversed()
        color_depth = cmap(depth_normalized)  # H,W,4

        # Set alpha to 0 where depth is 0
        color_depth[..., 3] = np.where(depth_normalized == 0, 0, color_depth[..., 3])

        # Convert color_depth from float [0,1] to uint8 [0,255] and remove alpha channel
        color_depth_rgb = (color_depth[..., :3] * 255).astype(np.uint8)

        # Use alpha channel for blending: where alpha==0, keep rgb_image pixel
        alpha_mask = color_depth[..., 3][..., None]
        if len(rgb_image.shape) == 2:
            overlay = (alpha * rgb_image[:, :, None].repeat(3, axis=2) + (1 - alpha) * color_depth_rgb).astype(np.uint8)
            overlay = np.where(alpha_mask == 0, rgb_image[:, :, None].repeat(3, axis=2), overlay)
        else:
            overlay = (alpha * rgb_image + (1 - alpha) * color_depth_rgb).astype(np.uint8)
            overlay = np.where(alpha_mask == 0, rgb_image, overlay)

        # Convert overlay to BGR for cv2 if needed
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        font_thickness = 2
        tag = "Depth Overlay"
        text_size, _ = cv2.getTextSize(tag, font, font_scale, font_thickness)
        text_x = overlay_bgr.shape[1] - text_size[0] - 10
        text_y = text_size[1] + 10
        cv2.putText(
            overlay_bgr,
            tag,
            (text_x, text_y),
            font,
            font_scale,
            (255, 255, 255),
            font_thickness,
            cv2.LINE_AA,
        )
        # Save again with tag
        output_path = str(self.output_folder / frame["depth_file_path"]).replace(".png", f"_overlay_{debug_tag}.png")
        cv2.imwrite(output_path, overlay_bgr)

    def project_lidar_to_camera(
        self, lidar_points, camera_intrinsics, lidar_to_camera_transform, image_width, image_height
    ):
        """Project LiDAR points onto camera image plane"""
        # Transform points from LiDAR frame to camera frame
        lidar_points_homo = np.hstack([lidar_points, np.ones((lidar_points.shape[0], 1))])
        camera_points_homo = (lidar_to_camera_transform @ lidar_points_homo.T).T
        camera_points = camera_points_homo[:, :3]

        # Filter points behind the camera
        valid_points = camera_points[:, 2] > 0
        camera_points = camera_points[valid_points]

        if len(camera_points) == 0:
            return np.full((image_height, image_width), -1, dtype=np.float32)

        # Project to image plane
        image_points = (camera_intrinsics @ camera_points.T).T
        image_points[:, 0] /= image_points[:, 2]
        image_points[:, 1] /= image_points[:, 2]

        # Filter points within image bounds
        valid_pixels = (
            (image_points[:, 0] >= 0)
            & (image_points[:, 0] < image_width)
            & (image_points[:, 1] >= 0)
            & (image_points[:, 1] < image_height)
        )

        valid_image_points = image_points[valid_pixels]
        valid_depths = camera_points[valid_pixels, 2]

        # Create depth image
        depth_image = np.full((image_height, image_width), -1, dtype=np.float32)

        if len(valid_image_points) > 0:
            pixel_coords = valid_image_points[:, :2].astype(int)
            for i, (x, y) in enumerate(pixel_coords):
                if 0 <= x < image_width and 0 <= y < image_height:
                    # Use closest depth if multiple points project to same pixel
                    if depth_image[y, x] == -1 or valid_depths[i] < depth_image[y, x]:
                        depth_image[y, x] = valid_depths[i]

        return depth_image

    def pointcloud2_to_xyz(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array of XYZ points"""
        points = []
        for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def find_closest_lidar_msgs(self, target_timestamp):
        closest_msgs = []
        for topic, lidar_msgs in self.lidar_msgs.items():
            n = self.lidar_accumulate_scans[topic]
            closest_msgs += self.find_n_closest_lidar_msg(target_timestamp, lidar_msgs, n=n)
        return closest_msgs

    def find_n_closest_lidar_msg(self, target_timestamp, lidar_msgs, n):
        """Find the n closestest LiDAR message to the target timestamp"""
        time_diffs = [abs(msg.header.stamp.to_sec() - target_timestamp.to_sec()) for msg in lidar_msgs]
        # return top n closest messages
        sorted_indices = np.argsort(time_diffs)[:n]
        return [lidar_msgs[i] for i in sorted_indices]

    def get_camera_intrinsics(self, frame):
        """Extract camera intrinsics from nerfstudio JSON"""
        fx = frame["fl_x"]
        fy = frame["fl_y"]
        cx = frame["cx"]
        cy = frame["cy"]

        return np.array([[float(fx), 0, float(cx)], [0, float(fy), float(cy)], [0, 0, 1]], dtype=np.float32)


class NerfstudioConverter:
    def __init__(self, camera_infos, camera_keys, tf_listener, config, data_folder, mission_name, head):
        self.blur_score = {}
        self.tf_listener = tf_listener
        self.camera_infos = camera_infos
        self.camera_keys = camera_keys
        self.config = config
        self.head = head

        self.taget_camera_infos = {k: None for k in camera_infos.keys()}
        self.map1 = {k: None for k in camera_infos.keys()}
        self.map2 = {k: None for k in camera_infos.keys()}
        self.invalid_mask = {k: None for k in camera_infos.keys()}

        self.output_folder = Path(data_folder) / f"{mission_name}_nerfstudio"
        self.output_folder.mkdir(parents=True, exist_ok=True)
        self.frames_json_file = self.output_folder / "transforms.json"

        self.images_folder = self.output_folder / "rgb"
        self.images_folder.mkdir(parents=True, exist_ok=True)

        self.frame_data = {"camera_model": "OPENCV", "frames": []}
        self.image_counters = {key: 0 for key in camera_keys.keys()}
        self.image_last_stored = {key: 0 for key in camera_keys.keys()}
        self.bridge = CvBridge()
        self.last_trans = None

        if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
            # Generate masks using Mask2Former for humans
            self.processor = Mask2FormerImageProcessor.from_pretrained("facebook/mask2former-swin-large-coco-panoptic")
            self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
                "facebook/mask2former-swin-large-coco-panoptic"
            )
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model.to(self.device)
            self.model.eval()
            self.mask_folder = self.output_folder / "mask"
            self.mask_folder.mkdir(parents=True, exist_ok=True)

        if self.config["nerfstudio"]["create_depth_based_on_lidar"]:
            self.depth_processor = DepthProcessor(self.config, self.tf_listener, self.output_folder, data_folder)

    def reset(self):
        self.last_trans = None

    def set_depth_config(self, active, camera=None, data_folder=None):
        self.use_depth_image = active
        if not active:
            return

        depth_img_bag_path = get_bag(camera["bag_pattern_depth_image"], directory=data_folder, try_until_suc=False)
        depth_info_bag_path = get_bag(camera["bag_pattern_depth_info"], directory=data_folder, try_until_suc=False)

        self.depth_info = None
        with rosbag.Bag(depth_info_bag_path, "r") as bag:
            for _, msg, _ in bag.read_messages(topics=[camera["depth_info_topic"]]):
                self.depth_info = msg
                break
        if self.depth_info is None:
            print(f"Failed to find depth info in bag {depth_info_bag_path}")
            exit(-1)

        self.depth_image_bag = rosbag.Bag(depth_img_bag_path, "r")
        self.depth_image_iterator = self.depth_image_bag.read_messages(topics=[camera["depth_image_topic"]])

    def __exit__(self, exc_type=None, exc_value=None, traceback=None):
        if hasattr(self, "depth_image_bag"):
            self.depth_image_bag.close()

    def process_frame(self, img_msg, topic, debug_tag, cfg):
        if self.image_counters[topic] >= self.config["num_images_per_topic"] or (
            self.head != -1 and self.image_counters[topic] >= self.head
        ):
            return False
        if img_msg.header.seq == 0:
            img_msg.header.seq = self.image_counters[topic]

        hz = cfg.get("hz", self.config["hz"])
        if img_msg.header.stamp.to_sec() - self.image_last_stored[topic] < (1 / hz) - 0.001:
            print(img_msg.header.seq, "freq")
            return True

        camera_key = self.camera_keys[topic]

        # Get transformation
        trans, quat_xyzw = self.tf_listener.lookupTransform(
            self.config["tf_config"]["world_frame"],
            img_msg.header.frame_id,
            img_msg.header.stamp,
            method=self.config["tf_config"]["tf_lookup_method"],
            odom_world=self.config["tf_config"]["odom_world"],
            odom_body=self.config["tf_config"]["odom_body"],
        )

        if trans is None or quat_xyzw is None:
            print(f"Warning: Could not get transform for {camera_key} at time {img_msg.header.stamp}")
            return True

        if self.last_trans is None:
            self.last_trans = trans

        if np.linalg.norm(np.array(self.last_trans[:2]) - np.array(trans[:2])) < cfg.get(
            "distance_threshold", self.config["distance_threshold"]
        ):
            print(img_msg.header.seq, "distance")
            return True

        self.last_trans = trans
        self.image_counters[topic] += 1
        self.image_last_stored[topic] = img_msg.header.stamp.to_sec()

        image_filename = f"{camera_key}_{img_msg.header.seq:05d}.png"
        image_path = self.images_folder / image_filename
        print(topic, self.image_counters[topic], image_path)

        if (
            self.config["nerfstudio"]["store"]
            or (not ("_rect" in topic) and self.taget_camera_infos[topic] is None)
            or ("_rect" in topic and self.camera_infos[topic] is None)
        ):
            # Convert and save image
            if isinstance(img_msg, CompressedImage) or "compressed" in topic:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
                # im_tensor = torch.from_numpy(cv_image).to(dtype=torch.float32) / 255.0
                # im_tensor = torch.flip(im_tensor, [-1])
            else:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
                # im_tensor = torch.from_numpy(cv_image).to(dtype=torch.float32) / 255.0

            # Undistort image if needed
            if "_rect" not in topic:
                # TODO add fisheye undistortion as paramter in cfg!
                (
                    cv_image,
                    self.taget_camera_infos[topic],
                    self.map1[topic],
                    self.map2[topic],
                    self.invalid_mask[topic],
                ) = undistort_image(
                    cv_image,
                    self.camera_infos[topic],
                    self.taget_camera_infos[topic],
                    self.map1[topic],
                    self.map2[topic],
                    self.invalid_mask[topic],
                    fisheye="zed2i" not in topic,
                )
                camera_info = self.taget_camera_infos[topic]
            else:
                camera_info = self.camera_infos[topic]
                if "zed2i" not in topic:
                    camera_info.D = [0, 0, 0, 0]
                else:
                    camera_info.D = [0, 0, 0, 0, 0]

            # Check blur and save image
            blur = cv2.Laplacian(cv_image, cv2.CV_64F).var()

            self.blur_score[image_filename] = blur
            if blur < cfg["blur_threshold"]:
                print(f"Warning: Image too blurry (blur value: {blur}). Skipping.")
                return True

            cv2.imwrite(str(image_path), cv_image)

            if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
                mask_file_path = str(image_path).replace("/rgb/", "/mask/")
                image = cv2.imread(str(image_path))
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(image_rgb)

                # Process image with Mask2Former
                inputs = self.processor(images=pil_image, return_tensors="pt")
                inputs = {k: v.to(self.device) for k, v in inputs.items()}

                with torch.no_grad():
                    outputs = self.model(**inputs)

                # Post-process the outputs to get segmentation maps
                predicted_segmentation_maps = self.processor.post_process_semantic_segmentation(
                    outputs, target_sizes=[pil_image.size[::-1]]
                )
                segmentation_map = predicted_segmentation_maps[0]
                human_mask = (segmentation_map == 0).cpu().numpy()  # Person class is 0
                binary_mask = (~human_mask * 255).astype(np.uint8)

                # Apply the mask from the rectification
                binary_mask[self.invalid_mask[topic]] = 0
                # Save the binary mask as PNG
                Image.fromarray(binary_mask, mode="L").save(mask_file_path)

                # Logging percentage human pixels
                human_pixel_count = np.sum(human_mask)
                total_pixels = human_mask.size
                coverage_percent = (human_pixel_count / total_pixels) * 100
                print(f"Human coverage in frame: {coverage_percent:.2f}%")

        else:
            if "_rect" not in topic:
                camera_info = self.taget_camera_infos[topic]
            else:
                camera_info = self.camera_infos[topic]
            if "zed2i" not in topic:
                camera_info.D = [0, 0, 0, 0]
            else:
                camera_info.D = [0, 0, 0, 0, 0]

        # Create transformation matrix using scipy Rotation
        try:
            rotation = R.from_quat(quat_xyzw).as_dcm()
        except:
            rotation = R.from_quat(quat_xyzw).as_matrix()
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation
        transform_matrix[:3, 3] = trans

        # Convert to OpenGL convention
        transform_matrix = ros_to_gl_transform(transform_matrix)

        # TODO check if these here need to be saved as strings or float and int is also good
        # Add frame data
        frame_data = {
            "file_path": f"./rgb/{image_filename}",
            "mask_file_path": f"./mask/{image_filename}",
            "transform_matrix": transform_matrix.tolist(),
            "camera_frame_id": img_msg.header.frame_id,
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
            "timestamp": str(img_msg.header.stamp.secs) + "_" + str(img_msg.header.stamp.nsecs),
        }

        if self.use_depth_image:
            while True:
                try:
                    depth_msg = next(self.depth_image_iterator)[1]
                    if depth_msg.header.seq == img_msg.header.seq:
                        break
                except StopIteration:
                    print("No more depth images available")
                    return True
            arr = np.frombuffer(depth_msg.data, dtype=np.uint8)
            png_bytes = arr[12:]  # skip 12-byte header
            depth_image = cv2.imdecode(png_bytes, cv2.IMREAD_UNCHANGED)
            depth_image_float = depth_image.astype(np.float32) / 1000.0  # Convert to meters
            depth_image_float[depth_image_float == 0] = 0
            stereo_depth_file_path = str(image_path).replace("/rgb/", "/stereo_depth/")
            stereo_depth_file_path = stereo_depth_file_path.replace(".jpg", ".png")
            frame_data["mask_path"] = f"./stereo_depth/{image_filename}"
            Path(stereo_depth_file_path).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(stereo_depth_file_path, depth_image_float)

        if self.config["nerfstudio"]["create_depth_based_on_lidar"]:
            frame_data = self.depth_processor.process(frame_data, debug_tag)

        if self.config["nerfstudio"]["create_mask_based_on_semantics"]:
            frame_data["mask_path"] = f"./mask/{image_filename}"

        self.frame_data["frames"].append(frame_data)
        return True

    def save_json(self):
        with open(self.frames_json_file, "w") as f:
            json.dump(self.frame_data, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Process ROS bags and extract camera images and transformations.")
    PATH = Path(WS) / "src/grand_tour_box/box_utils/box_converter/nerfstudio/cfg/grand_tour_release.yaml"
    parser.add_argument("--mission_name", type=str, default="2024-11-04-10-57-34", help="Mission Folder")
    parser.add_argument(
        "--mission_uuid",
        type=str,
        default="34c04ec5-c1b7-4674-9d64-99ade50f71d0",
        help="Mission UUID (uses GRI-1_release UUID if empty)",
    )
    # GRI-1 34c04ec5-c1b7-4674-9d64-99ade50f71d0
    # HAUS-1 10aa8311-2487-4e5c-9136-91b1d9ae4efa
    parser.add_argument("--config_file", type=str, default=PATH, help="Path to the configuration YAML file")
    parser.add_argument("--cluster", default=True, help="Flag to indicate if on or off")
    parser.add_argument("--head", type=int, default=-1, help="Number of images")

    args = parser.parse_args()

    if args.cluster:
        # /data = $TMPDIR local SSD
        # /out = $SCRACTH temporary storage

        if args.mission_uuid != "":
            mn = kleinkram.list_files(mission_ids=[args.mission_uuid])[0].mission_name
            MISSION_NAME = mn.replace("release_", "")
            uuid_release = args.mission_uuid
        else:
            uuid_release = get_uuid_mapping()[args.mission_name]["uuid_release"]
            MISSION_NAME = args.mission_name

        os.environ["KLEINKRAM_ACTIVE"] = "ACTIVE"
        os.environ["MISSION_UUID"] = uuid_release
        print(os.environ["MISSION_UUID"])
        print("Processing of mission started", MISSION_NAME)
        data_folder = Path(MISSION_DATA) / f"{MISSION_NAME}_nerfstudio"

    else:
        MISSION_NAME = args.mission_name
        data_folder = f"/tmp_disk/{MISSION_NAME}_nerfstudio"

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    Path(data_folder).mkdir(exist_ok=True, parents=True)

    try:
        print("Loading camera_info from for all images.")
        tf_bag_path = get_bag("*_tf_minimal.bag", directory=data_folder, try_until_suc=False)

        # Load camera infos
        camera_infos = {}
        camera_keys = {}
        for camera in config["cameras"]:
            bag_file = get_bag(camera["bag_pattern_info"], directory=data_folder, try_until_suc=False)

            with rosbag.Bag(bag_file, "r") as bag:
                for topic, msg, t in bag.read_messages(topics=[camera["info_topic"]]):
                    camera_infos[camera["image_topic"]] = msg
                    camera_keys[camera["image_topic"]] = camera["name"]
                    break
    except Exception as e:
        print("Failed loading data", e)
        # shutil.rmtree(data_folder, ignore_errors=True)
        exit(-1)

    # Initialize TF listener
    tf_listener = BagTfTransformer(tf_bag_path)

    # Initialize NerfstudioConverter
    converter = NerfstudioConverter(
        camera_infos, camera_keys, tf_listener, config, data_folder, MISSION_NAME, args.head
    )

    # Process image messages
    for _, camera in enumerate(config["cameras"]):
        bag_file = next(Path(data_folder).glob(camera["bag_pattern_image"]))
        with rosbag.Bag(bag_file, "r") as bag:
            start = bag.get_start_time() + config["skip_start_seconds"]
            end = bag.get_end_time() - config["skip_end_seconds"]
            if camera.get("use_depth_image", False):
                converter.set_depth_config(True, camera, data_folder)
            else:
                converter.set_depth_config(False)

            for topic, msg, t in bag.read_messages(
                topics=[camera["image_topic"]], start_time=rospy.Time(start), end_time=rospy.Time(end)
            ):
                # if msg.header.seq not in [619, 630, 1692, 226, 2332, 2869, 3610, 3728, 3850, 4675, 5862]:
                #     continue
                # store = copy.deepcopy(msg.header.stamp)
                # offset_in_ms = 0
                # for offset_in_ms in range(-100,100, 5):
                offset_in_ms = camera.get("offset_in_ms", 0.0)
                msg.header.stamp = msg.header.stamp + rospy.Duration.from_sec(offset_in_ms * 0.001)
                debug_tag = f"offset_{offset_in_ms}_ms"
                suc = converter.process_frame(msg, topic, debug_tag, camera)
                if not suc:
                    print(f"Finished processing {camera['name']}")
                    break
            converter.reset()

    # Save JSON file
    converter.save_json()

    # TAR files for cluster
    if args.cluster:
        out_dir = Path(converter.output_folder).stem
        parent_folder = Path(converter.output_folder).parent
        tar_file = Path(ARTIFACT_FOLDER) / "nerfstudio_scratch" / f"{MISSION_NAME}_nerfstudio.tar"
        tar_file.parent.mkdir(parents=True, exist_ok=True)
        print("Creating tar file", tar_file, " from ", out_dir, " in ", parent_folder)
        os.system(f"cd {parent_folder}; tar -cvf {tar_file} {out_dir}")

    # shutil.rmtree(data_folder, ignore_errors=True)


if __name__ == "__main__":
    main()
