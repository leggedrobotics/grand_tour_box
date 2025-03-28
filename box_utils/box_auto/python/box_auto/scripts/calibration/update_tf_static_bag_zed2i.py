from box_auto.utils import WS
import rosbag
from tf_bag import BagTfTransformer
from tf.transformations import quaternion_from_euler, quaternion_multiply
from math import pi
from pathlib import Path
import yaml
import copy

p = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration/"


for p in [str(s) for s in p.glob("*tf_static_metadata_*.bag") if "new" not in str(s)]:

    tf_listener = BagTfTransformer(p)
    with rosbag.Bag(p, "r") as bag:
        with rosbag.Bag(p.replace("tf_static_metadata_", "tf_static_metadata_new_"), "w", compression="lz4") as out_bag:

            for topic, msg, _t in bag.read_messages():
                if topic == "/tf_static":
                    delete_ids = []
                    for j, transform in enumerate(msg.transforms):
                        if "zed" in transform.header.frame_id or "zed" in transform.child_frame_id:
                            delete_ids.append(j)

                    msg.transforms = [transform for i, transform in enumerate(msg.transforms) if i not in delete_ids]
                    tf = tf_listener.lookupTransform("box_base", "zed_base_link", time=None, latest=True)

                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "box_base"
                    out.child_frame_id = "zed2i_base"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)

                    tf = tf_listener.lookupTransform("zed_base_link", "zed2i_imu_link", time=None, latest=True)
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "zed2i_base"
                    out.child_frame_id = "zed2i_imu"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)

                    # Rename frame_id + flip tf
                    tf = tf_listener.lookupTransform(
                        "zed_base_link", "zed2i_left_camera_optical_frame", time=None, latest=True
                    )
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "zed2i_base"
                    out.child_frame_id = "zed2i_right"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    rotation_180_z = quaternion_from_euler(0, 0, pi)
                    rot = quaternion_multiply(rotation_180_z, tf[1])
                    out.transform.rotation.x = rot[0]
                    out.transform.rotation.y = rot[1]
                    out.transform.rotation.z = rot[2]
                    out.transform.rotation.w = rot[3]

                    msg.transforms.append(out)

                    # Rename frame_id + flip tf
                    tf = tf_listener.lookupTransform(
                        "zed_base_link", "zed2i_right_camera_optical_frame", time=None, latest=True
                    )
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "zed2i_base"
                    out.child_frame_id = "zed2i_left"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]

                    rot = quaternion_multiply(rotation_180_z, tf[1])
                    out.transform.rotation.x = rot[0]
                    out.transform.rotation.y = rot[1]
                    out.transform.rotation.z = rot[2]
                    out.transform.rotation.w = rot[3]
                    msg.transforms.append(out)

                elif (
                    topic == "/gt_box/zed2i_driver_node/left_raw/image_raw_color/calibration_data"
                    or topic == "/gt_box/zed2i_driver_node/right_raw/image_raw_color/calibration_data"
                ):
                    cfg = yaml.safe_load(msg.data)
                    width, height = cfg["image_width"], cfg["image_height"]
                    cx, cy = cfg["camera_matrix"]["data"][2], cfg["camera_matrix"]["data"][5]
                    cfg["camera_matrix"]["data"][2] = width - cx - 1
                    cfg["camera_matrix"]["data"][5] = height - cy - 1

                    with open("/tmp/calib.yaml", "w") as f:
                        yaml.dump(cfg, f)
                    with open("/tmp/calib.yaml", "r") as f:
                        msg.data = f.read()

                out_bag.write(topic, msg, _t)
    break
