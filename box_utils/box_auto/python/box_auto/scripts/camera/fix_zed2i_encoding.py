import rosbag
from box_auto.utils import get_uuid_mapping, read_sheet_data
from pathlib import Path
import kleinkram
import shutil
import os
import numpy as np
import tqdm
import struct
import cv2
from sensor_msgs.msg import CompressedImage
import sys
from box_auto.utils import upload_simple

import pathlib

for parent in pathlib.Path(__file__).resolve().parents:
    bb = parent / "box_binding"
    if (bb / "compressed_depth").is_dir():
        sys.path.insert(0, str(bb))
        break

import compressed_depth


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

    # header = payload[12:20]
    # cols, rows = np.frombuffer(header, dtype=np.uint32)
    # log(f"  • RVL header : {cols}×{rows}")

    depth = compressed_depth.decode(payload, msg.format)
    # inspect_array(depth, "decoded")

    if not depth.flags["C_CONTIGUOUS"]:
        # log("    [WARN] Not C-contiguous – copying")
        depth = np.ascontiguousarray(depth)
    if not depth.flags["ALIGNED"]:
        # log("    [WARN] Not aligned – copying")
        depth = np.copy(depth)

    return depth


#
# Documentation:
# Why this script is needed:
# We initally compressed with RVL but this turned out to be a big pain in the ass without any benefits
# This changes to simply png encoding

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
_, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)
uuid_mappings = get_uuid_mapping()


# TODO After debugging set to true
OVERWRITE = True
topic_name = [
    "/boxi/zed2i/depth/image_raw/compressedDepth",
    "/boxi/zed2i/confidence/image_raw/compressedDepth",
    "/boxi/zed2i/depth/camera_info",
]
for name, data in uuid_mappings.items():
    try:
        # Create Temporary Directory
        tmp_dir = Path("/home/tutuna/Videos/fixing_zed2i_depth")
        tmp_dir.mkdir(parents=True, exist_ok=True)

        # Filter Good Missions.
        if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
            uuid_release = data["uuid_release"]
            res = kleinkram.list_files(
                mission_ids=[uuid_release],
                file_names=["*zed2i_depth.bag"],
            )
            ori_file = tmp_dir / res[0].name
            # Check if the original file exists, if not, download it
            if not ori_file.exists():
                kleinkram.download(
                    mission_ids=[uuid_release],
                    file_names=["*zed2i_depth.bag"],
                    dest=tmp_dir,
                    verbose=True,
                )
            else:
                print(f"Using existing file: {ori_file}")

            try:
                with rosbag.Bag(ori_file, "r") as count_bag:
                    total_msgs = sum(1 for _ in count_bag.read_messages(topics=topic_name))
            except Exception as e:
                raise RuntimeError(f"Failed to read messages from {ori_file}: {e}")

            if OVERWRITE:
                moved_file = tmp_dir / res[0].name.replace(".bag", "_tmp.bag")
                os.system(f"mv {ori_file} {moved_file}")
                ori_file = moved_file
                out_file = tmp_dir / res[0].name
            else:
                out_file = tmp_dir / res[0].name.replace(".bag", "_new.bag")

            print(f"Processing mission: {name}")
            with rosbag.Bag(ori_file, "r") as original_bag:
                with rosbag.Bag(out_file, "w") as out_bag:
                    for topic, msg, t in tqdm.tqdm(
                        original_bag.read_messages(topics=topic_name),
                        total=total_msgs,
                        desc="Decoding frames",
                        unit="frame",
                    ):
                        if topic in topic_name and topic != "/boxi/zed2i/depth/camera_info":
                            # try:
                            if "rvl" in msg.format:
                                # if topic.endswith("/compressedDepth"):
                                #     topic = topic.replace("/compressedDepth", "")
                                # topic = "/boxi/zed2i/depth/image_raw/compressedDepth"
                                # Original format: '32FC1; compressedDepth rvl'
                                depth_np = decode_depth_message(msg)
                                depth_mm = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
                                depth_mm = (depth_mm * 1000.0).astype(np.uint16)
                                # PNG encode using OpenCV
                                success, png_bytes = cv2.imencode(".png", depth_mm)
                                if not success:
                                    raise RuntimeError("PNG encoding failed")
                                png_bytes = png_bytes.tobytes()

                                new_msg = CompressedImage()
                                new_msg.header = msg.header
                                header = struct.pack("<IIf", depth_mm.shape[0], depth_mm.shape[1], 1000.0)
                                new_msg.data = header + png_bytes  # prepend header
                                new_msg.format = "16UC1; compressedDepth png"

                                # Write to the appropriate topic (you may want to adjust this)
                                out_bag.write(topic, new_msg, t)
                            else:
                                # If the message is not in RlVL format, write it unchanged
                                # This is useful for camera_info or other messages that are not depth images
                                out_bag.write(topic, msg, t)

                        else:
                            # Write unchanged other messages
                            out_bag.write(topic, msg, t)

            # ONLY USE THIS if you are really certain you want to overwrite the existing file
            if True:
                upload_simple(
                    project_name="GrandTour",
                    mission_name="release_" + name,
                    path=str(out_file),
                )
                print(f"Finished processing mission: {name}")
                if out_file.exists():
                    out_file.unlink()
                if ori_file.exists():
                    ori_file.unlink()

            # else:
            #     kleinkram.upload(
            #         mission_id=uuid_release,
            #         files=[out_file],
            #     )
            #     print(f"Finished processing mission: {name}")
            #     if out_file.exists():
            #         out_file.unlink()
            #     if ori_file.exists():
            #         ori_file.unlink()

    except Exception as e:
        print(f"Error processing mission {name}: {e}")
    finally:
        pass
        # TODO comment in to safe memory
        if False:
            shutil.rmtree(tmp_dir, ignore_errors=True)
