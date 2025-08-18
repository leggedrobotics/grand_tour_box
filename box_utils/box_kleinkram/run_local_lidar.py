# Script to process and upload lidar data for Grand Tour missions
# Handles downloading bag files, processing pointclouds, and uploading results

from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil

# Get mapping between mission names and UUIDs
uuid_mappings = get_uuid_mapping()
# Google spreadsheet containing mission information
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read mission data from spreadsheet
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

j = 0
# Iterate through all missions in the UUID mapping
for name, data in uuid_mappings.items():
    j += 1

    # Skip missions not found in spreadsheet
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    # Only process missions marked as good in the spreadsheet
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]

        # Check if mission has already been processed (has ready.bag file > 0.1MB)
        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*_ready.bag"])
        files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.1]

        if len(files) == 1:
            print(f"MISSION:   {name} already done")
            continue

        uuid = data["uuid"]

        try:
            # Create temporary directory for processing
            tmp_folder = Path("/data") / (name + "_lidar")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # Define bag files to download
            file_names = [
                "*_lpc_tf.bag",
                "*_tf_static_start_end.bag",
                "*_npc_velodyne.bag",
                "*_nuc_hesai_post_processed.bag",
                "*_nuc_livox.bag",
            ]

            # Download required bag files for processing
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            # Run pointcloud filtering in Docker container
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps filter_pointclouds; sleep 5;"
            )
            # Upload filtered pointclouds
            patterns = [
                "*_nuc_hesai_ready.bag",
                "*_nuc_livox_ready.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

            # Generate minimal TF static model without hesai_dlio.bag or ie_tc.bag
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps tf_static_minimal_model; sleep 5;"
            )

            # Run pointcloud undistortion using the minimal model
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps pointcloud_undistortion; sleep 5;"
            )

            # Upload undistorted pointclouds
            patterns = [
                "*_nuc_hesai_undist.bag",
                "*_nuc_livox_undist.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

            # Validate bag files have correct timestamps (after 2024)
            error = False
            for file_name in file_names:
                p = get_bag(file_name, directory=tmp_folder)
                import rosbag
                from datetime import datetime

                try:
                    with rosbag.Bag(p, "r") as bag:
                        start_time = bag.get_start_time()
                        start_datetime = datetime.fromtimestamp(start_time)
                        if start_datetime.year < 2024:
                            print(f"Error: Bag file {file_name} starts before 2024. Start time: {start_datetime}")
                            error = True
                            break
                except Exception as e:
                    print(f"Error reading bag file {file_name}: {e}")
            if error:
                print(f"Skipping mission {name} due to bag file errors.")
                continue

            # Process velodyne data
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps velodyne; sleep 5;"
            )
            # Filter velodyne pointclouds
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps filter_pointclouds_velo; sleep 5;"
            )
            # Upload processed velodyne data
            path_upload = get_bag("*_npc_velodyne_ready.bag", directory=tmp_folder)
            upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            # Copy important output files to backup location before cleanup
            try:
                patterns = [
                    "*_nuc_hesai_undist.bag",
                    "*_nuc_livox_undist.bag",
                    "*_nuc_hesai_ready.bag",
                    "*_nuc_livox_ready.bag",
                    "*_npc_velodyne_ready.bag",
                ]
                for p in patterns:
                    path = get_bag(p, directory=tmp_folder)
                    os.system(f"cp {p} /data/GrandTour/out_tmp")
            except Exception as e:
                print(e)
            # Clean up temporary directory
            shutil.rmtree(tmp_folder)
