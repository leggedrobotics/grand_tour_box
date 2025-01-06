from box_auto.utils import MISSION_DATA, BOX_AUTO_DIR, okviz_trajectory_to_bag, get_bag
import os
from pathlib import Path

if __name__ == "__main__":
    runs = {
        "alphasense_2_bw": {
            "data_folder": "euroc_grand_tour/alphasense/",
            "okvis_config": "okvis_config_2_bw.yaml",
        },
        # "alphasense_1_bw": {
        #     "data_folder": "euroc_grand_tour/alphasense/",
        #     "okvis_config": "okvis_config_1_bw.yaml",
        # },
        # "alphasense_1_rgb": {
        #     "data_folder": "euroc_grand_tour/alphasense/",
        #     "okvis_config": "okvis_config_1_rgb.yaml",
        # },
        # "alphasense_3_rgb": {
        #     "data_folder": "euroc_grand_tour/alphasense/",
        #     "okvis_config": "okvis_config_3_rgb.yaml",
        # },
        # "alphasense_5_all": {
        #     "data_folder": "euroc_grand_tour/alphasense/",
        #     "okvis_config": "okvis_config_5_all.yaml",
        # },
        # "hdr_3_rgb": {
        #     "data_folder": "euroc_grand_tour/hdr/",
        #     "okvis_config": "okvis_config_hdr_3rgb.yaml",
        # },
        # "hdr_1_rgb": {
        #     "data_folder": "euroc_grand_tour/hdr/",
        #     "okvis_config": "okvis_config_hdr_1rgb.yaml",
        # },
        # "zed_2_rgb": {
        #     "data_folder": "euroc_grand_tour/zed/",
        #     "okvis_config": "okvis_config_zed_2rgb.yaml",
        # },
        # "zed_1_rgb": {
        #     "data_folder": "euroc_grand_tour/zed/",
        #     "okvis_config": "okvis_config_zed_1rgb.yaml",
        # },
    }

    for k, v in runs.items():
        run_script = f"{BOX_AUTO_DIR}/docker/run.sh"

        relativ_euroc_data = v["data_folder"]
        relativ_okvis_config = v["data_folder"] + v["okvis_config"]
        command = f"/home/rsl/ros2_ws/build/okvis/okvis_app_synchronous  /tmp_disk/{relativ_okvis_config} /tmp_disk/{relativ_euroc_data}"
        cmd = run_script + f" --type=ros2 --mission-data={MISSION_DATA} --command='{command}'"

        # Run okvis using ROS2 Docker container
        print(cmd)
        os.system(cmd)

        # Convert trajectory into rosbag
        livox_bag_path = get_bag("*_nuc_livox.bag")
        input_csv = Path(MISSION_DATA) / v["data_folder"] / "okvis2-slam_trajectory.csv"
        output_bag = livox_bag_path.replace("_nuc_livox", f"_okvis_{k}")
        okviz_trajectory_to_bag(input_csv, output_bag, "okvis_world", "cpt7_imu")
