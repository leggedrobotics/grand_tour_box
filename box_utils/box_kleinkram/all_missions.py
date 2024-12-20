import os
import subprocess
import shlex

deployments = [
    # {
    #     "data_folder": "/media/jonfrey/BoxiS4-2TB/deployment_day_1",
    #     "mission_names": ["2024-10-01-11-29-55", "2024-10-01-11-47-44", "2024-10-01-12-00-49"],
    # },
    # {"data_folder": "/media/jonfrey/BoxiS4-2TB/deployment_day_3", "mission_names":  ["2024-10-18-12-50-31"]},
    # {
    #     "data_folder": "/media/jonfrey/BoxiS4-2TB/deployment_day_5",
    #     "mission_names": ["2024-10-29-09-08-34", "2024-10-29-09-53-44", "2024-10-29-16-11-44"],
    # },
    # {
    #     "data_folder": "/media/jonfrey/BoxiS4-2TB/deployment_day_6",
    #     "mission_names": ["2024-11-01-17-46-15"],
    # },
    # {
    #     "data_folder": "/media/jonfrey/BoxiS2-2TB/deployment_day_7",
    #     "mission_names": [
    #         "2024-11-02-17-10-25",
    #         "2024-11-02-17-43-10",
    #         "2024-11-02-17-18-32",
    #         "2024-11-02-19-47-44",
    #         "2024-11-02-20-03-21",
    #         "2024-11-02-20-26-48",
    #         "2024-11-02-21-12-51",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/BoxiS2-2TB/deployment_day_8",
    #     "mission_names": [
    #         "2024-11-03-07-52-45",
    #         "2024-11-03-07-57-34",
    #         "2024-11-03-08-17-23",
    #         "2024-11-03-08-42-30",
    #         "2024-11-03-11-03-50",
    #         "2024-11-03-13-51-43",
    #         "2024-11-03-13-59-54",
    #         "2024-11-03-14-36-54",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/BoxiS2-2TB/deployment_day_9",
    #     "mission_names": [
    #         "2024-11-04-10-57-34",
    #         "2024-11-04-13-07-13",
    #         "2024-11-04-14-55-02",
    #         "2024-11-04-11-56-53",
    #         "2024-11-04-14-19-11",
    #         "2024-11-04-16-05-00",
    #         "2024-11-04-12-41-45",
    #         "2024-11-04-16-52-38",
    #         "2024-11-04-12-55-59",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/T7/deployment_day_10",
    #     "mission_names": [
    #         "2024-11-05-20-10-15",
    #         "2024-11-05-20-13-11",
    #         "2024-11-05-20-21-14",
    #         "2024-11-05-20-31-24"
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/T7/deployment_day_11",
    #     "mission_names": [
    #         "2024-11-11-12-42-47",
    #         "2024-11-11-14-29-44",
    #         "2024-11-11-12-07-40",
    #         "2024-11-11-13-06-23",
    #         "2024-11-11-16-14-23",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/Untitled/deployment_day_12",
    #     "mission_names": [
    #         "2024-11-14-11-17-02",
    #         "2024-11-14-13-45-37",
    #         "2024-11-14-15-22-43",
    #         "2024-11-14-12-01-26",
    #         "2024-11-14-14-36-02",
    #         "2024-11-14-16-04-09",
    #     ],
    # },
    {
        "data_folder": "/media/jonfrey/Untitled/box_paper_datasets",
        "mission_names": [
            "2024-10-01-11-29-55",
            "2024-10-01-11-47-44",
            "2024-11-02-17-18-32",
            "2024-11-11-14-29-44",
            "2024-11-14-13-45-37",
        ],
    },
    # {
    #     "data_folder": "/media/jonfrey/T7/deployment_day_13",
    #     "mission_names": [
    #         "2024-11-15-10-16-35",
    #         "2024-11-15-12-06-03",
    #         "2024-11-15-15-07-36",
    #         "2024-11-15-11-18-14",
    #         "2024-11-15-13-56-45",
    #         "2024-11-15-16-41-14",
    #         "2024-11-15-11-37-15",
    #         "2024-11-15-14-14-12",
    #         "2024-11-15-14-43-52",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/BoxiS1-1TB/deployment_day_14",
    #     "mission_names": [
    #         "2024-11-18-11-42-04",
    #         "2024-11-18-12-05-01",
    #         "2024-11-18-13-22-14",
    #         "2024-11-18-13-48-19",
    #         "2024-11-18-15-46-05",
    #         "2024-11-18-16-45-27",
    #         "2024-11-18-16-59-23",
    #         "2024-11-18-17-13-09",
    #         "2024-11-18-17-31-36",
    #     ],
    # },
    # {"data_folder": "/media/jonfrey/BoxiS4-2TB/deployment_day_15", "mission_names": ["2024-11-21-16-36-19"]},
    # {
    #     "data_folder": "/media/jonfrey/Data/deployment_day_16",
    #     "mission_names": [
    #         "2024-11-25-11-44-05",
    #         "2024-11-25-14-57-08",
    #         "2024-11-25-11-56-26",
    #         "2024-11-25-16-02-15",
    #         "2024-11-25-13-52-08",
    #         "2024-11-25-16-36-19",
    #         "2024-11-25-14-01-49",
    #     ],
    # },
    # {
    #     "data_folder": "/media/jonfrey/Data/deployment_day_17",
    #     "mission_names": ["2024-12-03-11-09-06", "2024-12-03-13-15-38", "2024-12-03-13-26-40"],
    # },
    # {
    #     "data_folder": "/media/jonfrey/Data/deployment_day_18",
    #     "mission_names": ["2024-12-09-09-34-43", "2024-12-09-09-41-46", "2024-12-09-11-53-11", "2024-12-09-11-28-28"],
    # },
]


# Optional: Verbose mode with more details
def test_deployment_folders_verbose(deployments):
    """
    Detailed test of deployment folders with additional information.

    Args:
        deployments (list): List of deployment dictionaries

    Returns:
        dict: Detailed folder existence information
    """
    folder_status = {}

    for deployment in deployments:
        folder_path = deployment["data_folder"]
        folder_status[folder_path] = {
            "exists": os.path.exists(folder_path),
            "is_directory": os.path.isdir(folder_path) if os.path.exists(folder_path) else False,
            "mission_count": len(deployment["mission_names"]),
        }
    return folder_status


# Run verbose test
def main_verbose():
    verbose_results = test_deployment_folders_verbose(deployments)
    print("Deployment Folder Detailed Report:\n")
    for folder, status in verbose_results.items():
        print(f"Folder: {folder}")
        print(f"  Exists:        {'✅ Yes' if status['exists'] else '❌ No'}")
        print(f"  Is Directory:  {'✅ Yes' if status['is_directory'] else '❌ No'}")
        print(f"  Mission Count: {status['mission_count']}")
        print()


def execute_command_per_mission(cmd):
    summary = {}
    for dep in deployments:
        for mission_name in dep["mission_names"]:
            p = os.path.join(dep["data_folder"], mission_name)
            env = {"MISSION_DATA": str(p)}
            try:
                result = subprocess.run(
                    shlex.split(cmd),
                    env={**os.environ, **env},  # Merge the current environment with the custom one
                    check=True,  # Raise an exception if the command fails
                )
                exit_code = result.returncode
                summary[" ".join(p.split("/")[-2:])] = f"Script executed successfully with exit code {exit_code}."

                print(f"Script executed successfully with exit code {exit_code}.")
            except subprocess.CalledProcessError as e:
                print(f"Script failed with exit code {e.returncode}.")
                summary[" ".join(p.split("/")[-2:])] = f"Script failed with exit code {e.returncode}."
            except Exception as e:
                print(f"An error occurred: {e}")
                summary[" ".join(p.split("/")[-2:])] = f"An error occurred: {e}"

    for k, v in summary.items():
        print(k, ": ", v)


def process_all_gnss():
    # summary = {}
    # for dep in deployments:
    #     f = dep["data_folder"]
    #     cmd = f"python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/move_cpt7_files.py --data_folder {f} --cpt7_folder /media/jonfrey/Data/CPT7/2024-11-27_post_leica"
    #     result = subprocess.run(shlex.split(cmd))
    #     return_code = result.returncode
    #     summary[dep["data_folder"]] = return_code
    # for k, v in summary.items():
    #     print(k, ": ", v)

    # execute_command_per_mission(
    #     "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/export_raw_imu_bag.py"
    # )
    # execute_command_per_mission(
    #     "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/gnss_process.py"
    # )

    execute_command_per_mission(
        "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/export_gps_gt_trajectory_bag.py"
    )


# You can choose which test to run
if __name__ == "__main__":
    # Optional: Detailed test
    # main_verbose()

    # for dep in deployments:
    #     mission_names = " ".join(dep["mission_names"])
    #     data_folder = dep["data_folder"]
    #     os.system(f"python3 /home/jonfrey/git/grand_tour_box/box_utils/box_kleinkram/auto.py --mode=upload_and_process --data_folder={data_folder} --mission_names {mission_names}")
    # process_all_gnss()

    execute_command_per_mission("docker compose -f kleinkram.yaml up")

    # execute_command_per_mission(
    #     '/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/run.sh --type=kleinkram --command="export KLEINKRAM_ACTIVE=False; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/calibration/update_calibration.py"'
    # )

    # execute_command_per_mission(
    #     '/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/run.sh --type=kleinkram --command="export KLEINKRAM_ACTIVE=False; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/application/dlio.py"'
    # )
