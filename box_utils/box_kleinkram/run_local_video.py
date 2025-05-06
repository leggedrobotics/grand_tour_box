from box_auto.utils import get_uuid_mapping, read_sheet_data
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

go = False
for name, data in uuid_mappings.items():
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if name == "2024-11-18-13-22-14":
        go = True

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE" and go:
        uuid = data["uuid"]

        try:
            tmp_folder = Path("/tmp") / (name + "_video")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            file_names = [
                "*_jetson_hdr_front_anon.bag",
                "*_jetson_hdr_right_anon.bag",
                "*_jetson_hdr_left_anon.bag",
                "*_nuc_livox_ready.bag",
                "*_nuc_hesai_ready.bag",
                "*_lpc_state_estimator.bag",
                "*_tf_static_start_end.bag",
            ]

            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_converter/preview_video.py; sleep 5;"
            )
            video_path = tmp_folder / "youtube" / f"{name}.mp4"
            # try:
            #     upload_video(file_path=video_path, title=f"GrandTour Preview {name}")
            # except:
            #     pass

            if os.path.exists(video_path):
                os.system(f"cp {video_path} /home/jonfrey/Videos")
            else:
                print("Video not found: ", video_path)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
