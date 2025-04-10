from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

res_dir = Path("/home/jonfrey/git/grand_tour_box/box_utils/box_converter/cluster/.out/.submit")
res = {}
for name, data in uuid_mappings.items():
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid"]

        try:
            tmp_folder = Path("/tmp") / name
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid"]],
                file_names=["*hesai.bag"],
                dest=tmp_folder,
                verbose=True,
            )

            path = get_bag("*hesai.bag", directory=tmp_folder)
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps hesai; sleep 5;"
            )
            path_upload = get_bag("*hesai_post_processed.bag", directory=tmp_folder)
            upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir
