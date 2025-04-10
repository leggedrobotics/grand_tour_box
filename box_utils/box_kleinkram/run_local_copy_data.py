from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
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

        print("Processing Mission: ", name)
        master_p = [
            "*jetson_adis.bag",
            "*jetson_stim.bag",
            "*lpc_anymal_imu.bag",
            "*lpc_depth_cameras.bag",
            "*lpc_general.bag",
            "*lpc_locomotion.bag",
            "*lpc_state_estimator.bag",
            "*lpc_tf.bag",
            "*npc_depth_cameras.bag",
            "*npc_elevation_mapping.bag",
            "*npc_velodyne.bag",  # To publish packets
            "*nuc_hesai.bag",  # To publish packets
            "*nuc_livox.bag",  # To generate imu_intrinsics
        ]
        try:
            for pat in master_p:
                tmp_folder = Path("/tmp") / (name + "_run_local_copy_data")
                tmp_folder.mkdir(parents=True, exist_ok=True)
                patterns = [pat]

                # Upload GNSS directly upload to pub after merging PR

                # Generate tf_static_start_end from _pub
                # Generate hesai_postprocessed from _pub
                # Generate velodyne_postprocessed from _pub
                # Generate undistort from _pub (hesai, livox, velodyne)

                # Generate imu_intrinsics from _pub
                # Generate calib - from _pub (camera, tfs)
                # Generate dlio - from _pub (after undistort)
                # Generate tf_minimal tf_model from _pub (AFTER DLIO)
                # Generate IMU stim320 timeshift

                # Youtube video generation

                files = kleinkram.list_files(mission_ids=[data["uuid_pub"]], file_names=patterns)
                files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.1]
                if len(files) == 1:
                    # if files[0].status == "uploaded":
                    # TODO  implement this here

                    continue

                print(f"MISSION NAME      {name}:   {pat}: ", len(files))

                # make directory if not exists
                kleinkram.download(
                    mission_ids=[data["uuid"]],
                    file_names=patterns,
                    dest=tmp_folder,
                    verbose=True,
                )

                for pattern in patterns:
                    path_upload = get_bag(pattern, directory=tmp_folder)
                    upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            print("Error: ", e)
        finally:
            shutil.rmtree(tmp_folder)
