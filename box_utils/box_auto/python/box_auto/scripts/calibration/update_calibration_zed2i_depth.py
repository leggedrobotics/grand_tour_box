import rosbag

from pathlib import Path
from box_auto.utils import get_bag, upload_simple
import shutil
from box_auto.utils import get_uuid_mapping, read_sheet_data
import kleinkram

BAG_PATTERNS = [
    "*_jetson_zed2i_calib.bag",
    "*_jetson_zed2i_depth_new.bag",
]


raise ValueError(
    "We for now decided to not overwrite the camera info. Therefore script was never applied and also not fully tested. If you want to use it, please remove this line."
)


def update_depth_camera_info(tmp_folder, mission_name):
    bag_path_calib = get_bag(BAG_PATTERNS[0], directory=tmp_folder)
    bag_path_depth = get_bag(BAG_PATTERNS[1], directory=tmp_folder)

    with rosbag.Bag(bag_path_calib, "r") as inbag:
        camera_info_msg = next(inbag.read_messages(topics=["/boxi/zed2i/left/camera_info"]))

    with rosbag.Bag(bag_path_depth, "r") as inbag:
        out_bag = bag_path_depth.replace("_jetson_zed2i_depth_new.bag", "_jetson_zed2i_depth_calib.bag")

        with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
            for topic, msg, t in inbag.read_messages(topics=["/boxi/zed2i/depth/camera_info"]):
                new_msg = camera_info_msg
                new_msg.header = msg.header
                outbag.write(topic, new_msg, t)

        upload_simple("GrandTour", "pub_" + mission_name, out_bag, delete=True)


if __name__ == "__main__":
    SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
    uuid_mappings = get_uuid_mapping()
    topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)

    for name, data in uuid_mappings.items():
        try:
            tmp_folder = Path("/data") / (name + "_zed2i")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            if mission_data[name]["GOOD_MISSION"] != "TRUE":
                # print("Skip processing mission - no good mission")
                continue

            if name != "2024-11-04-10-57-34":
                print(f"Skipping mission {name} as it is not the target mission")
                continue

            res = kleinkram.list_files(
                mission_ids=[data["uuid_pub"]],
                file_names=BAG_PATTERNS,
            )
            if len(res) != 2:
                print("No zed2i bag found for mission", name)
                continue

            if [r.size for r in res if "depth_new" in r.name][0] < 500 * 1024 * 1024:  # less than 500 MB
                print(f"Skipping mission {name} as the bag is too small: {res[0].size / (1024 * 1024):.2f} MB")
                continue

            print(f"Processing mission {name} with bag {res[0].name}  {res[0].size}")
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=BAG_PATTERNS,
                dest=tmp_folder,
                verbose=True,
                overwrite=True,
            )
            update_depth_camera_info(tmp_folder, name)

            # exit(0)
        except Exception as e:
            raise RuntimeError(f"Error processing mission {name}: {e}") from e
        finally:
            # Clean up temporary folder
            try:
                if tmp_folder.exists():
                    shutil.rmtree(tmp_folder, name)
            except Exception as e:
                print(e)
