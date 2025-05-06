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

    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid_pub = data["uuid_pub"]

        files = kleinkram.list_files(mission_ids=[uuid_pub], file_names=["*_cpt7_raw_intrinsics.bag"])
        print(files)
        files = [f for f in files if f.size / 1024 < 5]

        if len(files) == 0:
            continue

        try:
            tmp_folder = Path("/tmp") / (name + "_intrinsics")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            file_names = [
                "*_jetson_stim.bag",
                "*_jetson_ap20_synced.bag",
                "*_nuc_livox.bag",
                "*_jetson_adis.bag",
                "*_cpt7_raw_imu.bag",
                "*_jetson_zed2i_prop.bag",
            ]
            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=[
                    "*_nuc_alphasense.bag",
                ],
                dest=tmp_folder,
                verbose=True,
            )

            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/calibration/update_imu_intrinsics.py;"
            )

            file_names = [
                "*_jetson_stim_intrinsics.bag",
                "*_jetson_ap20_intrinsics.bag",
                "*_nuc_livox_intrinsics.bag",
                "*_jetson_adis_intrinsics.bag",
                "*_cpt7_raw_intrinsics.bag",
                "*_nuc_alphasense_intrinsics.bag",
                "*_jetson_zed2i_prop_intrinsics.bag",
            ]
            for f in file_names:
                path_upload = get_bag(f, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir
