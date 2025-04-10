from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

for name, data in uuid_mappings.items():
    print("Mission name: ", name)

    if name not in ["2024-11-15-12-06-03", "2024-11-03-07-52-45"]:
        print("NO", name)
        continue

    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid"]

        try:
            tmp_folder = Path("/tmp") / name
            tmp_folder.mkdir(parents=True, exist_ok=True)

            file_names = [
                "*_jetson_adis.bag",
                "*_jetson_hdr_right_encoding.bag",
                "*_jetson_hdr_left_encoding.bag",
                "*_jetson_hdr_front_encoding.bag",
                "*_nuc_alphasense_color.bag",
            ]
            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            path = get_bag("*_jetson_adis.bag", directory=tmp_folder)
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/calibration/update_calibration.py;"
            )

            file_names = [
                "*_jetson_hdr_right_calib.bag",
                "*_jetson_hdr_left_calib.bag",
                "*_jetson_hdr_front_calib.bag",
                "*_nuc_alphasense_calib.bag",
            ]
            for f in file_names:
                path_upload = get_bag(f, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir
