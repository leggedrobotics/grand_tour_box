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

# Configure input and output paths
rosbags = {
    "imu0": ("/gt_box/alphasense_driver_node/imu", "*_nuc_alphasense_calib.bag"),
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
    "tf_static": ("tf_static", "*_tf_static.bag"),
}
output_dir = Path(MISSION_DATA) / "euroc_alphasense" / "grand_tour_box"

# Integrate all the measurements
tf_lookup = BagTfTransformer(get_bag(rosbags["tf_static"][1], rglob=False))

# Initialize CV Bridge
bridge = CvBridge()


def process_camera_data(topic, camera_info_topic, base_frame_id, bag_pattern, cam_folder):
    data_folder = cam_folder / "data"
    data_folder.mkdir(exist_ok=True, parents=True)
    csv_path = str(cam_folder / "data.csv")

    bag_path = get_bag(bag_pattern)

    with rosbag.Bag(bag_path, "r") as bag, open(csv_path, "w", newline="") as csvfile:
        for _topic, camera_info_msg, t in bag.read_messages(topics=[camera_info_topic]):
            t_sensor_base, q_sensor_base = tf_lookup.lookupTransform(base_frame_id, camera_info_msg.header.frame_id, t)
            T_sensor_base = np.eye(4)
            T_sensor_base[:3, :3] = R.from_quat(q_sensor_base).as_matrix()
            T_sensor_base[:3, 3] = np.array(t_sensor_base)
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

    return T_sensor_base, camera_info_msg


def process_imu_data(topic, bag_pattern, imu_folder):
    imu_folder.mkdir(exist_ok=True, parents=True)
    csv_path = str(imu_folder / "data.csv")
    bag_path = get_bag(bag_pattern)

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


# Process each bag
infos = {}
imu_frame_id = "TBD"  # Will be automaticially infered - IMU is processed before camera

for sensor, data in rosbags.items():
    folder = output_dir / sensor

    if "cam" in sensor:
        topic, camera_info_topic, bag_pattern = data
        T_sensor_base, camera_info_msg = process_camera_data(
            topic, camera_info_topic, imu_frame_id, bag_pattern, folder
        )
        infos[sensor] = {
            "T_sensor_base": T_sensor_base,
            "camera_info_msg": camera_info_msg,
        }
    elif "imu" in sensor:
        topic, bag_pattern = data
        imu_msg = process_imu_data(topic, bag_pattern, folder)
        imu_frame_id = imu_msg.header.frame_id
        infos[sensor] = {
            "imu_msg": imu_msg,
        }

print("Data conversion completed!")

# Create OKVIS config
template_file = Path(BOX_AUTO_DIR).parent / "box_converter" / "euroc" / "okvis.yaml"
updated_parameters = {
    k: {
        "T_SC": infos[k]["T_sensor_base"].flatten().tolist(),
        "image_dimension": [infos[k]["camera_info_msg"].width, infos[k]["camera_info_msg"].height],
        "distortion_coefficients": list(infos[k]["camera_info_msg"].D),
        "focal_length": [infos[k]["camera_info_msg"].K[0], infos[k]["camera_info_msg"].K[4]],
        "principal_point": [infos[k]["camera_info_msg"].K[2], infos[k]["camera_info_msg"].K[5]],
        "distortion_type": infos[k]["camera_info_msg"].distortion_model,
    }
    for k in ["cam0", "cam1"]
}

# Load the template YAML
with open(template_file) as stream:
    template_data = yaml.safe_load(stream)

# Update parameters
for i, cam_key in enumerate(["cam0", "cam1"]):
    for k in updated_parameters[cam_key].keys():
        template_data["cameras"][i][k] = updated_parameters[cam_key][k]


def write_opencv_yaml(filename, data_dict):
    """Garbage code to write yaml correctly to disk for formatting - required for okviz :/"""

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


write_opencv_yaml(output_dir / "okvis_config.yaml", template_data)

print(f"Created yaml for OKVIS (imu not updated) {output_dir}/okvis_config.yaml")
