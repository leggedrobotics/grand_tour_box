# Import necessary libraries and utility functions
from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

# Get mapping between names and UUIDs
uuid_mappings = get_uuid_mapping()
# Google Sheets ID containing mission data
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Retrieve data from the spreadsheet
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

# Counter for tracking missions processed
j = 0
# Iterate through all mission mappings
for name, data in uuid_mappings.items():
    j += 1

    # Skip the first three missions
    if j < 4:
        continue

    # Check if mission exists in spreadsheet data
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    # Only process missions marked as good
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]
        # List all ready bag files for this mission
        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*_ready.bag"])
        # Filter out small files (less than 0.1 MB)
        files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.1]

        # Skip if mission is already processed (has 3 files)
        if len(files) == 3:
            print(f"MISSION:   {name} already done")
            continue

        uuid = data["uuid"]

        try:
            # Create temporary folder for processing
            tmp_folder = Path("/tmp") / name
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # Define bag files to download
            file_names = ["*_tf_static_start_end.bag", "*_nuc_hesai_ready.bag", "*_cpt7_raw_imu.bag"]

            # Download required files to temporary folder
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            # Run Docker container to process the data
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps tf_static_minimal_model; sleep 5;"
            )

            # Define output files to upload
            patterns = [
                "*_dlio_hesai.bag",
            ]
            # Upload processed files
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            # Handle any errors during processing
            print("Error: ", e)
        finally:
            # Clean up temporary files
            shutil.rmtree(tmp_folder)
