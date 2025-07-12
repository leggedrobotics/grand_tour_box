import rosbag
from box_auto.utils import get_uuid_mapping, read_sheet_data
from pathlib import Path
import kleinkram
import shutil

raise ValueError("This script is not needed anymore. The anonymization has been fixed in the original script.")
# Documentation:
# Why this scirpt is needed.
# In the previous version of the anonymization script, the header information was not preserved.
# Instead the rosbag_write time was written as the header and the seq number was set to 0.
# This script fixes the anonymization by restoring the original header information.
# Reads from the _pub and writes to the _anon_new.bag files.
# Afterwards we have to rerun the upload_clean_mission_hdr_with_flip.py (for the HDR)
# Also the header frame_id in the HDR images and camera info is wrong

CONFIG = {
    "{}_nuc_alphasense_color.bag": (
        [
            "/gt_box/alphasense_driver_node/cam1/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam2/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam3/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam4/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam5/color_corrected/image/compressed",
        ],
        "{}_nuc_alphasense_anon.bag",
    ),
    "{}_jetson_hdr_left_encoding.bag": (
        [
            "/gt_box/hdr_left/image_raw/compressed",
        ],
        "{}_jetson_hdr_left_anon.bag",
    ),
    "{}_jetson_hdr_front_encoding.bag": (
        [
            "/gt_box/hdr_front/image_raw/compressed",
        ],
        "{}_jetson_hdr_front_anon.bag",
    ),
    "{}_jetson_hdr_right_encoding.bag": (
        [
            "/gt_box/hdr_right/image_raw/compressed",
        ],
        "{}_jetson_hdr_right_anon.bag",
    ),
    # "{}_jetson_zed2i_images.bag": (
    #     [
    #         "/boxi/zed2i/left/image_raw/compressed",
    #         "/boxi/zed2i/right/image_raw/compressed",
    #     ],
    #     "{}_jetson_zed2i_images_anon.bag",
    # ),
}

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)
uuid_mappings = get_uuid_mapping()

for name, data in uuid_mappings.items():
    try:
        tmp_dir = Path("/data/my_life_sucks")
        tmp_dir.mkdir(parents=True, exist_ok=True)
        if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
            uuid = data["uuid_pub"]

            # replace in all of config {} with the name
            config_filled = {k.format(name): v for k, v in CONFIG.items()}
            anon_files = [v[1].format(name) for v in config_filled.values()]
            anon_files_updated = [k.replace(".bag", "_new.bag") for k in anon_files]

            # Check if the anonymized files already exist
            res = kleinkram.list_files(
                mission_ids=[uuid],
                file_names=anon_files_updated,
            )
            # if len(res) == len(anon_files_updated):
            #     print(f"All anonymized files already exist for mission {name}. Skipping...")
            #     continue

            # Download the original files
            kleinkram.download(
                mission_ids=[uuid],
                file_names=list(config_filled.keys()),
                dest=tmp_dir,
                verbose=True,
            )

            # Download anonymize the files
            print(config_filled.keys(), anon_files)
            kleinkram.download(
                mission_ids=[uuid],
                file_names=anon_files,
                dest=tmp_dir,
                verbose=True,
            )

            for original_file, (topics, anon_file) in config_filled.items():
                print(f"Processing {original_file} for mission {name}...")
                original_path = tmp_dir / original_file
                anon_path = tmp_dir / anon_file.format(name)
                anon_path_new = str(anon_path).replace(".bag", "_new.bag")

                headers = {topic: {} for topic in topics}

                with rosbag.Bag(original_path, "r") as original_bag:
                    for topic, msg, t in original_bag.read_messages(topics=topics):
                        ts_in_ns = t.secs * 1_000_000_000 + t.nsecs
                        headers[topic][ts_in_ns] = {
                            "header": msg.header,
                            "seq": msg.header.seq,
                            "stamp": msg.header.stamp,
                            "frame_id": msg.header.frame_id,
                        }

                with rosbag.Bag(anon_path, "r") as original_bag:
                    with rosbag.Bag(anon_path_new, "w", compression="lz4") as anon_bag:
                        for topic, msg, t in original_bag.read_messages():
                            if topic in topics:
                                ns_key = t.secs * 1_000_000_000 + t.nsecs
                                msg.header.stamp = headers[topic][ns_key]["stamp"]
                                msg.header.seq = headers[topic][ns_key]["seq"]
                                msg.header.frame_id = headers[topic][ns_key]["frame_id"]
                            anon_bag.write(topic, msg, t)

                files = kleinkram.list_files(mission_ids=[uuid], file_names=[str(Path(anon_path_new).name)])

                if len(files) == 1:
                    print("delete the file id")
                    kleinkram.delete_files(file_ids=[f.id for f in files])

                kleinkram.upload(
                    mission_id=uuid,
                    files=[anon_path_new],
                )
    except Exception as e:
        print(f"Error processing mission {name}: {e}")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
