# Import required libraries and utility functions
from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

# Get mapping between mission names and UUIDs
uuid_mappings = get_uuid_mapping()
# Google Spreadsheet ID containing mission data
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read data from the spreadsheet
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

# Loop through each mission in the UUID mappings
for name, data in uuid_mappings.items():
    print("Mission name: ", name)

    # Skip missions not found in the spreadsheet
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    # Process only missions marked as "GOOD_MISSION"
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid_pub = data["uuid_pub"]

        # List intrinsics bag files for this mission
        files = kleinkram.list_files(mission_ids=[uuid_pub], file_names=["*_cpt7_raw_intrinsics.bag"])
        print(files)
        # Filter out files larger than 5KB
        files = [f for f in files if f.size / 1024 < 5]

        # Skip if no valid files found
        if len(files) == 0:
            continue

        try:
            # Create temporary folder for processing
            tmp_folder = Path("/tmp") / (name + "_intrinsics")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # Define list of bag files to download
            file_names = [
                "*_jetson_stim.bag",
                "*_jetson_ap20_synced.bag",
                "*_nuc_livox.bag",
                "*_jetson_adis.bag",
                "*_cpt7_raw_imu.bag",
                "*_jetson_zed2i_prop.bag",
            ]
            # Download the main sensor bag files
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )
            # Download alphasense bag files separately
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=[
                    "*_nuc_alphasense.bag",
                ],
                dest=tmp_folder,
                verbose=True,
            )

            # Run the intrinsics calibration script
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/calibration/update_imu_intrinsics.py;"
            )

            # Define output intrinsics files to upload
            file_names = [
                "*_jetson_stim_intrinsics.bag",
                "*_jetson_ap20_intrinsics.bag",
                "*_nuc_livox_intrinsics.bag",
                "*_jetson_adis_intrinsics.bag",
                "*_cpt7_raw_intrinsics.bag",
                "*_nuc_alphasense_intrinsics.bag",
                "*_jetson_zed2i_prop_intrinsics.bag",
            ]
            # Upload each calibrated intrinsics file
            for f in file_names:
                path_upload = get_bag(f, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            # Clean up temporary directory
            shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir
