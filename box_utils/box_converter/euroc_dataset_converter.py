import rosbag
import cv2
from cv_bridge import CvBridge
import csv
from box_auto.utils import get_bag, MISSION_DATA, BOX_AUTO_DIR
from pathlib import Path
from tf_bag import BagTfTransformer
from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml

# Initialize CV Bridge
bridge = CvBridge()


def process_camera_data(topic, camera_info_topic, bag_pattern, cam_folder):
    data_folder = cam_folder / "data"
    data_folder.mkdir(exist_ok=True, parents=True)
    csv_path = str(cam_folder / "data.csv")

    bag_path = get_bag(bag_pattern, rglob=False)

    with rosbag.Bag(bag_path, "r") as bag, open(csv_path, "w", newline="") as csvfile:
        for _topic, camera_info_msg, t in bag.read_messages(topics=[camera_info_topic]):
            break

        writer = csv.writer(csvfile)
        writer.writerow(["#timestamp [ns]", "filename"])
        for _topic, msg, _t in bag.read_messages(topics=[topic]):
            timestamp = msg.header.stamp.to_nsec()
            filename = f"{timestamp}.png"
            filepath = data_folder / filename

            # Convert ROS image to OpenCV image and save
            cv_image = bridge.compressed_imgmsg_to_cv2(msg)  # bgr8 - desired_encoding="passtrough"
            cv2.imwrite(str(filepath), cv_image)

            writer.writerow([timestamp, filename])

    return camera_info_msg


def process_imu_data(topic, bag_pattern, imu_folder):
    imu_folder.mkdir(exist_ok=True, parents=True)
    csv_path = str(imu_folder / "data.csv")
    bag_path = get_bag(bag_pattern, rglob=False)

    with rosbag.Bag(bag_path, "r") as bag, open(csv_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ]
        )

        for _topic, msg, _t in bag.read_messages(topics=[topic]):
            timestamp = msg.header.stamp.to_nsec()
            angular_velocity = msg.angular_velocity
            linear_acceleration = msg.linear_acceleration
            writer.writerow(
                [
                    timestamp,
                    angular_velocity.x,
                    angular_velocity.y,
                    angular_velocity.z,
                    linear_acceleration.x,
                    linear_acceleration.y,
                    linear_acceleration.z,
                ]
            )
    return msg


def write_opencv_yaml(filename, data_dict):
    """Garbage code to write yaml correctly to disk for formatting - required for okvis :/"""

    def write_node(fs, node, name):
        """
        Recursively write nodes to the FileStorage object

        Parameters:
        fs: OpenCV FileStorage object
        node: Data to write (can be dict, list, or value)
        name: Node name in YAML
        """
        if isinstance(node, dict):
            # Create a nested mapping node
            _ = fs.startWriteStruct(name, cv2.FileNode_MAP)
            for key, value in node.items():
                write_node(fs, value, key)
            fs.endWriteStruct()
        elif isinstance(node, (list, tuple)):
            # Create a sequence node
            _ = fs.startWriteStruct(name, cv2.FileNode_SEQ)
            for item in node:
                if isinstance(item, dict):
                    # For dictionaries inside sequences
                    _ = fs.startWriteStruct(None, cv2.FileNode_MAP)
                    for key, value in item.items():
                        write_node(fs, value, key)
                    fs.endWriteStruct()
                else:
                    fs.write(None, item)
            fs.endWriteStruct()
        else:
            # Write leaf node
            if isinstance(node, bool):
                # Convert boolean to integer (0 or 1) as OpenCV expects
                if node:
                    fs.write(name, "true")
                else:
                    fs.write(name, "false")
            else:
                fs.write(name, node)

    # Create a FileStorage object
    fs = cv2.FileStorage(filename, cv2.FileStorage_WRITE)
    # Write the root dictionary
    for key, value in data_dict.items():
        write_node(fs, value, key)

    # Release the FileStorage object
    fs.release()


def process(rosbags, tag):
    # Configure input and output paths
    output_dir = Path(MISSION_DATA) / "euroc_grand_tour" / tag
    cams = [k for k in rosbags.keys() if "cam" in k]
    imus = [k for k in rosbags.keys() if "imu" in k]

    # Integrate all the measurements
    tf_lookup = BagTfTransformer(get_bag(rosbags["tf_static"][1], rglob=False))

    # Process each bag
    infos = {}

    for sensor, data in rosbags.items():
        folder = output_dir / sensor

        if "cam" in sensor:
            topic, camera_info_topic, bag_pattern = data
            camera_info_msg = process_camera_data(topic, camera_info_topic, bag_pattern, folder)
            infos[sensor] = {
                "camera_info_msg": camera_info_msg,
            }
        elif "imu" in sensor:
            topic, bag_pattern = data
            imu_msg = process_imu_data(topic, bag_pattern, folder)
            infos[sensor] = {
                "imu_msg": imu_msg,
            }

    print("Data conversion completed!")

    # Generate for each IMU an OKVIZ config
    for imu in imus:
        # Create OKVIS config
        template_file = Path(BOX_AUTO_DIR).parent / "box_converter" / "euroc" / f"okvis_{tag}.yaml"
        updated_parameters = []

        for k in cams:
            t_sensor_base, q_sensor_base = tf_lookup.lookupTransform(
                infos[imu].header.frame_id, infos[k]["camera_info_msg"].header.frame_id
            )
            T_sensor_base = np.eye(4)
            T_sensor_base[:3, :3] = R.from_quat(q_sensor_base).as_matrix()
            T_sensor_base[:3, 3] = np.array(t_sensor_base)

            updated_parameters[k] = {
                "T_SC": T_sensor_base.flatten().tolist(),
                "image_dimension": [infos[k]["camera_info_msg"].width, infos[k]["camera_info_msg"].height],
                "distortion_coefficients": list(infos[k]["camera_info_msg"].D),
                "focal_length": [infos[k]["camera_info_msg"].K[0], infos[k]["camera_info_msg"].K[4]],
                "principal_point": [infos[k]["camera_info_msg"].K[2], infos[k]["camera_info_msg"].K[5]],
                "distortion_type": infos[k]["camera_info_msg"].distortion_model.replace("radtan", "radialtangential"),
            }

        # Load the template YAML
        with open(template_file) as stream:
            template_data = yaml.safe_load(stream)

        # Update parameters
        for i, cam_key in enumerate(cams):
            for k in updated_parameters[cam_key].keys():
                template_data["cameras"][i][k] = updated_parameters[cam_key][k]

        write_opencv_yaml(output_dir / f"okvis_config_{imu}.yaml", template_data)
        print(f"Created yaml for OKVIS (imu not updated) {output_dir}/okvis_config_{imu}.yaml")

        euroc_cfg_path = str(output_dir / "euroc_config.yaml")
        # Write the dictionary to the YAML file
        with open(euroc_cfg_path, "w") as f:
            yaml.dump(rosbags, f, default_flow_style=False)


if __name__ == "__main__":
    all_cameras_cfg = {
        "imu0": ("/gt_box/cpt7/offline_from_novatel_logs/imu", "*_cpt7_raw_imu.bag"),
        "imu1": ("/gt_box/alphasense_driver_node/imu", "*_nuc_alphasense_calib.bag"),
        "imu2": ("/gt_box/ap20/imu", "*_jetson_ap20_synced.bag"),
        "imu3": ("/gt_box/zed2i/zed_node/imu_raw/data", "*_jetson_zed2i_prop.bag"),
        "imu4": ("/gt_box/stim320/imu", "*_jetson_stim.bag"),
        "imu5": ("/gt_box/adis16475_node/imu", "*_jetson_adis.bag"),
        "imu6": ("/gt_box/livox/imu_si_compliant", "*_nuc_livox.bag"),
        "cam0": (
            "/gt_box/alphasense_driver_node/cam2/color/image/compressed",
            "/gt_box/alphasense_driver_node/cam2/color/camera_info",
            "*_nuc_alphasense_calib.bag",
        ),  # Right
        "cam1": (
            "/gt_box/alphasense_driver_node/cam3/color/image/compressed",
            "/gt_box/alphasense_driver_node/cam3/color/camera_info",
            "*_nuc_alphasense_calib.bag",
        ),  # Left
        "cam2": (
            "/gt_box/alphasense_driver_node/cam1/color/image/compressed",
            "/gt_box/alphasense_driver_node/cam1/color/camera_info",
            "*_nuc_alphasense_calib.bag",
        ),  # Color
        "cam3": (
            "/gt_box/alphasense_driver_node/cam4/color/image/compressed",
            "/gt_box/alphasense_driver_node/cam4/color/camera_info",
            "*_nuc_alphasense_calib.bag",
        ),  # Color
        "cam4": (
            "/gt_box/alphasense_driver_node/cam5/color/image/compressed",
            "/gt_box/alphasense_driver_node/cam5/color/camera_info",
            "*_nuc_alphasense_calib.bag",
        ),  # Color
        "cam5": (
            "/gt_box/hdr_left/image_raw/compressed",
            "/gt_box/hdr_left/camera_info",
            "*_jetson_hdr_left_calib.bag",
        ),
        "cam6": (
            "/gt_box/hdr_front/image_raw/compressed",
            "/gt_box/hdr_front/camera_info",
            "*_jetson_hdr_front_calib.bag",
        ),
        "cam7": (
            "/gt_box/hdr_right/image_raw/compressed",
            "/gt_box/hdr_right/camera_info",
            "*_jetson_hdr_right_calib.bag",
        ),
        "cam8": (
            "/gt_box/zed2i/zed_node/left/image_rect_color/compressed",
            "/gt_box/zed2i/zed_node/left/camera_info",
            "*_jetson_zed2i_images_calib.bag",
        ),
        "cam9": (
            "/gt_box/zed2i/zed_node/right/image_rect_color/compressed",
            "/gt_box/zed2i/zed_node/right/camera_info",
            "*_jetson_zed2i_images_calib.bag",
        ),
        "tf_static": ("tf_static", "*_tf_static.bag"),
    }
    process(all_cameras_cfg, "all_cameras")
