from box_auto.utils import get_uuid_mapping, deployments, read_sheet_data, upload_simple
import os
from pathlib import Path
import time
import shutil

# Define the spreadsheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)

UUID_MAPPING = get_uuid_mapping()


def run_and_upload(ori_files, mission_name):
    try:
        tmp_folder = f"/tmp/{mission_name}"
        if os.path.exists(tmp_folder):
            shutil.rmtree(tmp_folder)

        os.mkdir(tmp_folder)
        for f in ori_files:
            if str(f).endswith(".mcap") and "hdr" in str(f.parent):
                tmpdir = tmp_folder + "/" + f.parent.stem
                os.mkdir(tmpdir)
                shutil.copytree(str(f.parent), tmpdir, dirs_exist_ok=True)
                # This copies all mcap files including metadata!
            else:
                shutil.copy(str(f), tmp_folder)

        time.sleep(3)
        if str(ori_files[0]).endswith(".mcap"):
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps hdr_convert; sleep 5;"
            )

        time.sleep(1)
        if mission_data[mission_name]["hdr_camera_sync"] == "TRUE":
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/hdr_timestamp_adjuster.py; sleep 2"
            )
        else:
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/hdr_timestamp_adjuster.py --skip_update; sleep 2"
            )

        time.sleep(1)
        os.system(
            f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/update_hdr_compression.py; sleep 2"
        )

        time.sleep(1)
        upload_files = [str(b) for b in Path(tmp_folder).rglob(f"*{m}_jetson_hdr_*_encoding.bag")]
        if len(upload_files) != 3:
            print(f"{mission_name}: Not all files were created")
            return

        print(f"{mission_name}: Successfully created all files and uploading to kleinkram")
        time.sleep(1)

        mission_name = "pub_" + mission_name

        for path in upload_files:
            upload_simple(project_name="GrandTour", mission_name=mission_name, path=path)

    except Exception as e:
        print(f"{mission_name}: Error: {e}")
    finally:
        print("CleanUp tmp folder:")
        shutil.rmtree(tmp_folder)


for d in deployments:
    for m in d["mission_names"]:
        MISSION_DATA = os.path.join(d["data_folder"], m)

        print(f"Mission: {m}")

        jetson_mcap_files = list(Path("/media/jonfrey/Untitled/jetson_data_save/").rglob(f"*{m}_jetson_hdr_*.mcap"))
        mission_mcap_files = list(Path(MISSION_DATA).rglob(f"*{m}_jetson_hdr_*.mcap"))
        mission_bag_files = [
            b
            for b in Path(MISSION_DATA).rglob(f"*{m}_jetson_hdr_*.bag")
            if "update" not in str(b) and "rect" not in str(b) and "calib" not in str(b) and "encoding" not in str(b)
        ]

        print(f"Jetson Files: {jetson_mcap_files}")
        print(f"Mission Files: {mission_mcap_files}")
        print(f"Mission Bag Files: {mission_bag_files}")

        if not len(jetson_mcap_files) == 3 and not len(mission_mcap_files) == 3 and not len(mission_bag_files) == 3:
            raise ValueError(f"{m}: No original bag files available!")

        if len(jetson_mcap_files) == 3:
            ori_files = jetson_mcap_files
        elif len(mission_mcap_files) == 3:
            ori_files = mission_mcap_files
        elif len(mission_bag_files) == 3:
            ori_files = mission_bag_files

        run_and_upload(ori_files, m)
