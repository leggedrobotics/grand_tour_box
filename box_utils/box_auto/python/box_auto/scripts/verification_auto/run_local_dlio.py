from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

j = 0
for name, data in uuid_mappings.items():
    j += 1

    # if j < 4:
    #     continue

    if (
        name != "2024-10-01-11-29-55"
        and name != "2024-10-01-11-47-44"
        and name != "2024-10-01-12-00-49"
        and name != "2024-10-29-09-53-44"
    ):
        continue

    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]
        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*_nuc_hesai_ready.bag"])
        files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.1]

        if len(files) == 3:
            print(f"MISSION:   {name} already done")
            continue

        if len(files) == 0:
            print(f"MISSION:   {name} not found")
            continue

        uuid = data["uuid"]

        try:
            tmp_folder = Path("/tmp") / name
            tmp_folder.mkdir(parents=True, exist_ok=True)

            file_names = ["*_tf_static_start_end.bag", "*_nuc_hesai_ready.bag", "*_cpt7_raw_imu.bag"]

            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            os.system(
                f"export MISSION_DATA={tmp_folder}; cd /home/tutuna/box_ws; source devel/setup.bash; python3 /home/tutuna/box_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/application/dlio.py; sleep 5;"
            )

            patterns = [
                "*_hesai_dlio.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)
                os.remove(path_upload)  # Remove the file after upload

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
