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

    if j < 4:
        continue

    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]
        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*_ready.bag"])
        files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.1]

        if len(files) == 3:
            print(f"MISSION:   {name} already done")
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
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps tf_static_minimal_model; sleep 5;"
            )

            patterns = [
                "*_dlio_hesai.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
