import os
import json
import cv2
import numpy as np
from mcap.writer import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# Paths
FOLDER = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/box_binding/zed2i_depth_frames/rgb"
MAPFILE = os.path.join(FOLDER, "rgb_image_map.json")
OUT_FOLDER = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/box_binding"
OUTFILE = os.path.join(OUT_FOLDER, "images_with_camerainfo.mcap")
CAMERA_INFO_J = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/box_binding/zed2i_depth_frames/camera_info.json"

IMAGE_TOPIC = "/camera/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera_info"
FRAME_ID = "zed2i_left_camera_optical_frame"
ENCODING = "bgr8"

with open(CAMERA_INFO_J, "r") as f:
    cam = json.load(f)["camera"]
w, h = int(cam["w"]), int(cam["h"])
fx, fy, cx, cy = map(float, (cam["fx"], cam["fy"], cam["cx"], cam["cy"]))

HEADER_DEFINITION = """
builtin_interfaces/Time stamp
string frame_id
"""

TIME_DEFINITION = """
int32 sec
uint32 nanosec
"""

CAMERAINFO_DEFINITION = """
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
RegionOfInterest roi
"""

REGIONOFINTEREST_DEFINITION = """
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
"""

IMAGE_DEFINITION = """
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
"""

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(get_types_from_msg(TIME_DEFINITION, "builtin_interfaces/msg/Time"))
typestore.register(get_types_from_msg(HEADER_DEFINITION, "std_msgs/msg/Header"))
typestore.register(get_types_from_msg(REGIONOFINTEREST_DEFINITION, "sensor_msgs/msg/RegionOfInterest"))
typestore.register(get_types_from_msg(CAMERAINFO_DEFINITION, "sensor_msgs/msg/CameraInfo"))
typestore.register(get_types_from_msg(IMAGE_DEFINITION, "sensor_msgs/msg/Image"))

CameraInfo = typestore.types["sensor_msgs/msg/CameraInfo"]
Image = typestore.types["sensor_msgs/msg/Image"]
Header = typestore.types["std_msgs/msg/Header"]
Time = typestore.types["builtin_interfaces/msg/Time"]
RegionOfInterest = typestore.types["sensor_msgs/msg/RegionOfInterest"]

# Calibration matrices as np.ndarray
K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=np.float64)
R = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64)
P = np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0], dtype=np.float64)
D = np.array([], dtype=np.float64)  # no distortion
binning_x = binning_y = 0
roi_defaults = dict(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False)

with open(MAPFILE, "r") as f:
    ts_map = json.load(f)
files = sorted(f for f in os.listdir(FOLDER) if f.endswith(".png") and f in ts_map)

with open(OUTFILE, "wb") as f:
    writer = Writer(f)
    writer.start("ros2")
    image_schema_id = writer.register_schema(
        name="sensor_msgs/msg/Image", encoding="ros2msg", data=IMAGE_DEFINITION.encode()
    )
    caminfo_schema_id = writer.register_schema(
        name="sensor_msgs/msg/CameraInfo", encoding="ros2msg", data=CAMERAINFO_DEFINITION.encode()
    )
    image_channel_id = writer.register_channel(
        topic=IMAGE_TOPIC, message_encoding="cdr", schema_id=image_schema_id, metadata={}
    )
    caminfo_channel_id = writer.register_channel(
        topic=CAMERA_INFO_TOPIC, message_encoding="cdr", schema_id=caminfo_schema_id, metadata={}
    )

    for fname in files:
        img = cv2.imread(os.path.join(FOLDER, fname))
        if img is None:
            print(f"Skipped unreadable image {fname}")
            continue

        ns = int(ts_map[fname] * 1e9)
        sec = ns // 10**9
        nsec = ns % 10**9
        hh, ww, _ = img.shape

        stamp = Time(sec, nsec)
        header = Header(stamp, FRAME_ID)
        imsg = Image(header, hh, ww, ENCODING, 0, ww * 3, np.ascontiguousarray(img.reshape(-1).astype(np.uint8)))
        writer.add_message(
            channel_id=image_channel_id,
            log_time=ns,
            data=typestore.serialize_cdr(imsg, "sensor_msgs/msg/Image"),
            publish_time=ns,
        )

        roi = RegionOfInterest(
            roi_defaults["x_offset"],
            roi_defaults["y_offset"],
            roi_defaults["height"],
            roi_defaults["width"],
            roi_defaults["do_rectify"],
        )
        cmsg = CameraInfo(
            header,
            hh,
            ww,
            "plumb_bob",
            D,  # float64[] as np.ndarray
            K,  # float64[9] as np.ndarray
            R,  # float64[9] as np.ndarray
            P,  # float64[12] as np.ndarray
            binning_x,
            binning_y,
            roi,
        )
        writer.add_message(
            channel_id=caminfo_channel_id,
            log_time=ns,
            data=typestore.serialize_cdr(cmsg, "sensor_msgs/msg/CameraInfo"),
            publish_time=ns,
        )

    writer.finish()
