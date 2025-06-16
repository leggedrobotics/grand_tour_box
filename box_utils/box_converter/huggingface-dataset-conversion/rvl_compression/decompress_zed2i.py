import os
import numpy as np
import rosbag
import cv2
from tqdm import tqdm
import json
import collections


import compressed_depth

PRINT_VERBOSE = False


def log(*args):
    if PRINT_VERBOSE:
        print(*args)


def process_rgb_images(bag_path, topic_name, output_dir, process_one=False):
    os.makedirs(output_dir, exist_ok=True)
    frame_idx = 0
    image_map = collections.OrderedDict()

    # Count messages
    with rosbag.Bag(bag_path, "r") as count_bag:
        total_msgs = sum(1 for _ in count_bag.read_messages(topics=[topic_name]))

    print(f"\nOpening bag: {bag_path}")
    print(f"RGB Topic: {topic_name}")
    print(f"Messages to process: {total_msgs}")

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in tqdm(
            bag.read_messages(topics=[topic_name]), total=total_msgs, desc="Decoding RGB frames", unit="frame"
        ):
            try:
                img_np = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if img_np is None:
                    raise RuntimeError("Failed to decode compressed image")

                img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
                file_name = f"frame_{frame_idx:04d}.png"
                output_path = os.path.join(output_dir, file_name)
                cv2.imwrite(output_path, cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
                # Map filename to header timestamp (as float)
                image_map[file_name] = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

            except Exception as exc:
                print("  • ERROR      :", type(exc).__name__, exc)

            frame_idx += 1
            if process_one:
                break

    # Write JSON mapping
    image_map_path = os.path.join(output_dir, "rgb_image_map.json")
    with open(image_map_path, "w") as f:
        json.dump(image_map, f, indent=2)
    print(f"Saved RGB image mapping to {image_map_path}")
    print("\nFinished RGB frames processed:", frame_idx)


def save_camera_info_json(bag_path, camera_info_topic, output_path):

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[camera_info_topic]):
            if msg._type == "sensor_msgs/CameraInfo":
                w = msg.width
                h = msg.height
                fx = msg.K[0]
                fy = msg.K[4]
                cx = msg.K[2]
                cy = msg.K[5]
                scale = 1000

                data = {"camera": {"w": w, "h": h, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "scale": scale}}

                with open(output_path, "w") as f:
                    json.dump(data, f, indent=4)
                print(f"Saved camera info to {output_path}")
                break
        else:
            print(f"No camera info found on topic '{camera_info_topic}' in bag '{bag_path}'")


def inspect_array(arr, label="array"):
    if not PRINT_VERBOSE:
        return
    print(f"  • {label:<10} : shape={arr.shape}, dtype={arr.dtype}")
    print(f"    ├ type      : {type(arr)}")
    print(f"    ├ ndim      : {arr.ndim}")
    print(f"    ├ itemsize  : {arr.itemsize}")
    print(f"    ├ strides   : {arr.strides}")
    print(f"    ├ size      : {arr.size}")
    print(f"    ├ nbytes    : {arr.nbytes}")
    print(f"    ├ flags     : {arr.flags}")
    print(f"    └ first val : {arr.flat[0] if arr.size > 0 else 'n/a'}")


def decode_depth_message(msg, verbose_payload=False):
    payload = np.frombuffer(msg.data, dtype=np.uint8).copy()
    if verbose_payload:
        print(f"[VERBOSE] Payload size: {payload.size} bytes")
        print(f"[VERBOSE] First 64 bytes: {payload[:64]}")
        print(f"[VERBOSE] Last 64 bytes: {payload[-64:]}")
        print(f"[VERBOSE] Payload dtype: {payload.dtype}")
        print(f"[VERBOSE] Payload shape: {payload.shape}")

    if payload.size < 20:
        raise ValueError("Message too small (need 20+ bytes)")

    header = payload[12:20]
    cols, rows = np.frombuffer(header, dtype=np.uint32)
    log(f"  • RVL header : {cols}×{rows}")

    depth = compressed_depth.decode(payload, msg.format)
    inspect_array(depth, "decoded")

    if not depth.flags["C_CONTIGUOUS"]:
        log("    [WARN] Not C-contiguous – copying")
        depth = np.ascontiguousarray(depth)
    if not depth.flags["ALIGNED"]:
        log("    [WARN] Not aligned – copying")
        depth = np.copy(depth)

    return depth


def save_depth_image(depth_np, output_path):
    depth_cv = np.asarray(depth_np)
    assert depth_cv.dtype == np.float32

    depth_mm = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
    depth_mm = (depth_mm * 1000.0).astype(np.uint16)

    if cv2.imwrite(output_path, depth_mm):
        log("  • saved      :", output_path)
    else:
        log("  • save FAILED:", output_path)


def process_bag(bag_path, topic_name, output_dir, process_one=False):
    os.makedirs(output_dir, exist_ok=True)
    frame_idx = 0
    image_map = collections.OrderedDict()

    with rosbag.Bag(bag_path, "r") as count_bag:
        total_msgs = sum(1 for _ in count_bag.read_messages(topics=[topic_name]))

    print(f"\nOpening bag: {bag_path}")
    print(f"Topic: {topic_name}")
    print(f"Messages to process: {total_msgs}")

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in tqdm(
            bag.read_messages(topics=[topic_name]), total=total_msgs, desc="Decoding frames", unit="frame"
        ):
            try:
                depth_np = decode_depth_message(msg)
                file_name = f"frame_{frame_idx:04d}.png"
                output_path = os.path.join(output_dir, file_name)
                save_depth_image(depth_np, output_path)
                # Map filename to header timestamp (as float)
                image_map[file_name] = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
            except Exception as exc:
                print("  • ERROR      :", type(exc).__name__, exc)

            frame_idx += 1
            if process_one:
                break

    # Write JSON mapping
    image_map_path = os.path.join(output_dir, "depth_image_map.json")
    with open(image_map_path, "w") as f:
        json.dump(image_map, f, indent=2)
    print(f"Saved depth image mapping to {image_map_path}")
    print("\nFinished – frames processed:", frame_idx)


if __name__ == "__main__":

    bag_path = "/home/tutuna/Videos/radar_svo/" "zed2i_depth.bag"

    topic_name = "/boxi/zed2i/depth/image_raw/compressedDepth"
    camera_info_topic = "/boxi/zed2i/depth/camera_info"

    output_dir = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/" "box_binding/zed2i_depth_frames/depth"
    cam_json_output_dir = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/" "box_binding/zed2i_depth_frames"

    process_bag(bag_path, topic_name, output_dir, process_one=False)
    # Save camera info once, consistent with depth PNGs (scale=1000)
    camera_json_path = os.path.join(cam_json_output_dir, "camera_info.json")
    save_camera_info_json(bag_path, camera_info_topic, camera_json_path)

    rgb_bag_path = "/home/tutuna/Videos/radar_svo/" "zed2i_images.bag"
    rgb_topic = "/boxi/zed2i/left/image_rect_color/compressed"
    output_dir_rgb = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/" "box_binding/zed2i_depth_frames/rgb"

    process_rgb_images(rgb_bag_path, rgb_topic, output_dir_rgb, process_one=False)
