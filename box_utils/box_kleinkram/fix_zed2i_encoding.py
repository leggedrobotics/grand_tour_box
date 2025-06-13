import rosbag
from box_auto.utils import get_uuid_mapping, read_sheet_data
from pathlib import Path
import kleinkram
import shutil
import os
from box_auto.utils import upload_simple

#
# Documentation:
# Why this script is needed:
# We initally compressed with RVL but this turned out to be a big pain in the ass without any benefits
# This changes to simply png encoding

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)
uuid_mappings = get_uuid_mapping()


# TODO After debugging set to true
OVERWRITE = False

for name, data in uuid_mappings.items():
    try:
        tmp_dir = Path("/data/large_ssd_folder")
        tmp_dir.mkdir(parents=True, exist_ok=True)
        if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
            uuid_release = data["uuid_release"]
            res = kleinkram.list_files(
                mission_ids=[uuid_release],
                file_names=["*zed2i_depth.bag"],
            )
            kleinkram.download(
                mission_ids=[uuid_release],
                file_names=["*zed2i_depth.bag"],
                dest=tmp_dir,
                verbose=True,
            )
            ori_file = tmp_dir / res[0].name

            if OVERWRITE:
                moved_file = tmp_dir / res[0].name.replace(".bag", "_tmp.bag")
                os.system(f"mv {ori_file} {moved_file}")
                ori_file = moved_file
                out_file = tmp_dir / res[0].name
            else:
                out_file = tmp_dir / res[0].name.replace(".bag", "_new.bag")

            with rosbag.Bag(ori_file, "r") as original_bag:
                with rosbag.Bag(out_file, "w") as out_bag:
                    for topic, msg, t in original_bag.read_messages():
                        if topic == "/boxi/zed2i/depth/image_raw/compressedDepth":
                            # I would change topic name to align with ANYmal - "/anymal/depth_camera/front_lower/depth/image_rect_raw"
                            topic = "/boxi/zed2i/depth/image_raw"
                            # TODO update message
                            msg

                        out_bag.write(topic, msg, t)

            # ONLY USE THIS if you are really certain you want to overwrite the existing file
            if False:
                upload_simple(
                    project_name="GrandTour",
                    mission_name="release_" + name,
                    path=out_file,
                )
            else:
                kleinkram.upload(
                    mission_id=uuid_release,
                    files=[out_file],
                )

    except Exception as e:
        print(f"Error processing mission {name}: {e}")
    finally:
        pass
        # TODO comment in to safe memory
        if False:
            shutil.rmtree(tmp_dir, ignore_errors=True)
