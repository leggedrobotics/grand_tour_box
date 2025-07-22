# Import necessary libraries
from box_auto.utils import get_uuid_mapping, read_sheet_data
import os
from pathlib import Path
import kleinkram
import shutil

# Get mapping of mission names to UUIDs
uuid_mappings = get_uuid_mapping()
# Google spreadsheet ID containing mission data
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read mission data from the spreadsheet
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

# Define output directory for results
res_dir = Path("/home/jonfrey/git/grand_tour_box/box_utils/box_converter/cluster/.out/.submit")
res = {}

# Flag to control processing start point
go = False
for name, data in uuid_mappings.items():
    # Skip missions not found in spreadsheet data
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    # Start processing from this specific mission
    if name == "2024-11-18-13-22-14":
        go = True

    # Process only good missions after the start point is reached
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE" and go:
        uuid = data["uuid"]

        try:
            # Create temporary folder for mission data
            tmp_folder = Path("/tmp") / (name + "_video")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # Define bag file patterns to download
            file_names = [
                "*_jetson_hdr_front_anon.bag",
                "*_jetson_hdr_right_anon.bag",
                "*_jetson_hdr_left_anon.bag",
                "*_nuc_livox_ready.bag",
                "*_nuc_hesai_ready.bag",
                "*_lpc_state_estimator.bag",
                "*_tf_static_start_end.bag",
            ]

            # Download required files for this mission
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            # Run the video generation script
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_converter/preview_video.py; sleep 5;"
            )
            # Path to the generated video file
            video_path = tmp_folder / "youtube" / f"{name}.mp4"
            # YouTube upload functionality (commented out)
            # try:
            #     upload_video(file_path=video_path, title=f"GrandTour Preview {name}")
            # except:
            #     pass

            # Copy the video to Videos directory if it exists
            if os.path.exists(video_path):
                os.system(f"cp {video_path} /home/jonfrey/Videos")
            else:
                print("Video not found: ", video_path)

        except Exception as e:
            print("Error: ", e)
        finally:
            # Clean up temporary directory
            shutil.rmtree(tmp_folder)
