from box_auto.utils import get_uuid_mapping, read_sheet_data
import os
from pathlib import Path
import kleinkram
import shutil

# source ~/.venv/wavemap/bin/activate; box_ws; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_kleinkram/run_local_wavemap_render.py
uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

j = 0
for name, data in uuid_mappings.items():
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        if j == 0:
            j += 1
            continue
        uuid = data["uuid_release"]
        files = kleinkram.list_files(mission_ids=[uuid])
        if len(files) != 34:
            print(f"MISSION:   {name} do not process")
            continue
        print(files, len(files))

        try:
            print("Start running wavemap")
            tmp_folder = Path("/data") / (name + "_wavemap_create")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            artifact_folder = Path("/data") / "depth_scratch_new" / (name + "_nerfstudio")
            artifact_folder.mkdir(parents=True, exist_ok=True)

            map_file = Path("/data") / "wavemap_scratch" / "wavemap" / (name + "_wavemap.wvmp")

            os.system(
                f"export MISSION_DATA={tmp_folder}; export KLEINKRAM_ACTIVE=ACTIVE; export MISSION_UUID={uuid}; export ARTIFACT_FOLDER={artifact_folder}; /home/jonfrey/.venv/wavemap/bin/python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/application/wavemap_render_depth.py --map_file={map_file};"
            )
        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
