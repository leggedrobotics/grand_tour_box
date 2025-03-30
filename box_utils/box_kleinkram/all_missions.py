import os
import subprocess
import shlex
from box_auto.utils import deployments


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
    k = 0
    for dep in deployments:
        for j, mission_name in enumerate(dep["mission_names"]):
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


def execute_command_per_mission2(cmd, parallel=1, skip=-1):
    summary = {}
    j = 0
    executed_j = 0

    for dep in deployments:
        for mission_name in dep["mission_names"]:
            if j < skip:
                j += 1
                continue

            p = os.path.join(dep["data_folder"], mission_name)
            env = {"MISSION_DATA": str(p)}

            try:
                cmd_out = cmd
                if (executed_j - parallel + 1) % parallel != 0:  # TRUST ME BRO
                    cmd_out = cmd + "&"
                    os.system(f"export MISSION_DATA={str(p)}; {cmd_out}")
                    summary[" ".join(p.split("/")[-2:])] = "Results not logged given that executed in background"
                else:
                    result = subprocess.run(
                        shlex.split(cmd_out),
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

            executed_j += 1
            j += 1

    for k, v in summary.items():
        print(k, ": ", v)


def process_all_gnss():
    summary = {}
    for dep in deployments:
        f = dep["data_folder"]
        cmd = f"python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/move_cpt7_files.py --data_folder {f} --cpt7_folder /media/jonfrey/Data/CPT7/2024-11-27_post_leica"
        result = subprocess.run(shlex.split(cmd))
        return_code = result.returncode
        summary[dep["data_folder"]] = return_code
    for k, v in summary.items():
        print(k, ": ", v)

    execute_command_per_mission(
        "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/export_raw_imu_bag.py"
    )
    execute_command_per_mission(
        "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/gnss_process.py"
    )


if __name__ == "__main__":
    # process_all_gnss()

    # execute_command_per_mission(
    #     "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/cpt7/export_gps_gt_trajectory_bag.py"
    # )

    # execute_command_per_mission(
    #     "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/ap20/ap20_python_online_matcher.py"
    # )
    # execute_command_per_mission(
    #     "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/ap20/ap20_validate.py"
    # )

    execute_command_per_mission(
        "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/calibration/update_calibration.py"
    )

    # execute_command_per_mission2( "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/color_correction.py", parallel = 20, skip = 0)
    # execute_command_per_mission("docker compose -f kleinkram.yaml up")

    # execute_command_per_mission(
    #     '/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/run.sh --type=kleinkram --command="export KLEINKRAM_ACTIVE=False; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/application/dlio.py"'
    # )
