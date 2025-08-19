from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram

# Get mapping of mission names to UUIDs
uuid_mappings = get_uuid_mapping()
# Google Spreadsheet ID containing mission data
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read mission data from the spreadsheet
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

# Iterate through all missions in the UUID mappings
for name, data in uuid_mappings.items():
    print("Mission name: ", name)

    # Skip missions not found in the spreadsheet data
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    # Process only missions marked as "GOOD_MISSION" in the spreadsheet
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid"]

        try:
            # Create temporary folder for processing this mission
            tmp_folder = Path("/data") / (name + "_tf_minimal_model")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # Define bag files to download
            file_names = ["*_lpc_tf.bag", "*_tf_static_start_end.bag", "*_cpt7_ie_tc.bag", "*_hesai_dlio.bag"]

            # Download required bag files for the mission
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            # Run the transformation preparation script
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/special/prepare_tf_transforms_for_public.py;"
            )

            # Define output files to upload
            file_names = [
                "*_tf_minimal.bag",
                "*_tf_model.bag",
            ]
            # Upload processed bag files to the project
            for f in file_names:
                path_upload = get_bag(f, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            pass
            # shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir
        # break
