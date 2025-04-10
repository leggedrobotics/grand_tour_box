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
            tmp_folder = Path("/tmp") / (name + "_lidar")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            file_names = [
                "*_lpc_tf.bag",
                "*_tf_static_start_end.bag",
                "*_npc_velodyne.bag",
                "*_nuc_hesai_post_processed.bag",
                "*_nuc_livox.bag",
            ]

            # make directory if not exists
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=file_names,
                dest=tmp_folder,
                verbose=True,
            )

            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps filter_pointclouds; sleep 5;"
            )
            patterns = [
                "*_nuc_hesai_ready.bag",
                "*_nuc_livox_ready.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

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

            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps velodyne; sleep 5;"
            )
            path_upload = get_bag("*_npc_velodyne_ready.bag", directory=tmp_folder)
            upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

            # Doggy minimal generation without fusing hesai_dlio.bag nore ie_tc.bag as intermediate solution
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps tf_static_minimal_model; sleep 5;"
            )

            # This requires the minimal
            os.system(
                f"export MISSION_DATA={tmp_folder}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/kleinkram.yaml up --no-deps pointcloud_undistortion; sleep 5;"
            )

            patterns = [
                "*_nuc_hesai_undist.bag",
                "*_nuc_livox_undist.bag",
            ]
            for pattern in patterns:
                path_upload = get_bag(pattern, directory=tmp_folder)
                upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
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
            shutil.rmtree(tmp_folder)
