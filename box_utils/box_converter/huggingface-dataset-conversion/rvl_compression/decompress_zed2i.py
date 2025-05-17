import os
import numpy as np
import rosbag
import cv2
from tqdm import tqdm

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "box_binding")))
import compressed_depth

PRINT_VERBOSE = False


def log(*args):
    if PRINT_VERBOSE:
        print(*args)


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


def decode_depth_message(msg):
    payload = np.frombuffer(msg.data, dtype=np.uint8).copy()
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

    with rosbag.Bag(bag_path, "r") as count_bag:
        total_msgs = sum(1 for _ in count_bag.read_messages(topics=[topic_name]))

    print(f"\nOpening bag: {bag_path}")
    print(f"Topic: {topic_name}")
    print(f"Messages to process: {total_msgs}")

    # Second pass: process messages
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in tqdm(
            bag.read_messages(topics=[topic_name]), total=total_msgs, desc="Decoding frames", unit="frame"
        ):
            try:
                depth_np = decode_depth_message(msg)
                output_path = os.path.join(output_dir, f"frame_{frame_idx:04d}.png")
                save_depth_image(depth_np, output_path)
            except Exception as exc:
                print("  • ERROR      :", type(exc).__name__, exc)

            frame_idx += 1
            if process_one:
                break

    print("\nFinished – frames processed:", frame_idx)


if __name__ == "__main__":
    bag_path = "/home/tutuna/Videos/2024-11-14-13-45-37_heap_testsite/" "2024-11-14-13-45-37_jetson_zed2i_depth.bag"

    topic_name = "/boxi/zed2i/depth/image_raw/compressedDepth"

    output_dir = "/home/tutuna/box_ws/src/grand_tour_box/box_utils/" "box_binding/zed2i_depth_frames"

    process_bag(bag_path, topic_name, output_dir, process_one=False)
