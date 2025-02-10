from box_auto.utils import MISSION_DATA, BOX_AUTO_DIR, okviz_trajectory_to_bag, get_bag
from pathlib import Path
import yaml
import datetime

# Assumes you have previously run the euroc_dataset_converter.py
# After generating the dataset one has to still manually adjust the okvis config to match the desired profile
# For running the IMU experiment we recommend copying the data for the different imus in subfolder
# euroc_grand_tour
#  |euroc_config.yaml
#  |--alphasense_imu
#      |--cpt7
#         |--cam0
#         |--cam1
#         |--imu0
#      |--stim
#         |--cam0
#         |--cam1
#         |--imu0
#      ....
# For this create the subfolder e.g. "stim" and ensure to rename the correct imu in this case imu4
# Refer for this to the euroc_config.yaml
# Do not forget to manually modify the correct okvis_config_alphasense_2_bw_stim.yaml files


if __name__ == "__main__":
    runs = {
        "imu0_alphasense_2_bw": {
            "data_folder": "euroc_grand_tour/imu0_alphasense_2_bw/",
            "okvis_config": "okvis_config_imu0.yaml",
        },
    }

    for k, v in runs.items():
        run_script = f"{BOX_AUTO_DIR}/docker/run.sh"

        relativ_euroc_data = v["data_folder"]
        relativ_okvis_config = v["data_folder"] + v["okvis_config"]
        command = f"/home/rsl/ros2_ws/build/okvis/okvis_app_synchronous  /tmp_disk/{relativ_okvis_config} /tmp_disk/{relativ_euroc_data}"

        cmd = run_script + f" --type=ros2 --mission-data={MISSION_DATA} --command='{command}'"
        # Run okvis using ROS2 Docker container
        # print(cmd)
        # os.system(cmd)
        print("We assume Okvis was run successfully")

        metadata = {}
        okvis_config_path = Path(MISSION_DATA) / (v["data_folder"] + v["okvis_config"])
        with open(okvis_config_path, "r") as f:
            metadata["okvis_config"] = f.read()

        euroc_config_path = Path(MISSION_DATA) / v["data_folder"] / "euroc_config.yaml"
        with open(euroc_config_path, "r") as f:
            metadata["euroc_config"] = f.read()

        metadata["date"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        metadata_string = yaml.dump(metadata, default_flow_style=False)

        # Convert trajectory into rosbag
        livox_bag_path = get_bag("*_nuc_livox.bag")
        input_csv = Path(MISSION_DATA) / v["data_folder"] / "okvis2-slam_trajectory.csv"
        output_bag = livox_bag_path.replace("_nuc_livox", f"_okvis_{k}")
        okviz_trajectory_to_bag(
            input_csv, output_bag, "okvis_world", "cpt7_imu", metadata_string, "/gt_box/okvis/offline_metadata"
        )
