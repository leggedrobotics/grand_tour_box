import os
import subprocess
import shlex

base = "/media/jonfrey/Untitled/box_paper_dataset_v2"
missions = [
    "2024-10-01-11-29-55_polybahn",
    #  "2024-10-01-11-47-44_main_building_hall",
    "2024-11-02-17-18-32_sphinx_walking_stairs_2",
    "2024-11-03-13-51-43_eigergletscher_hike_down",
    "2024-11-14-13-45-37_heap_testsite",
    "2024-11-25-16-36-19_leica_warehouse_groundfloor",
    # "2024-11-11-12-42-47_pilatus_kulm_hike",
    "2024-11-18-13-22-14_arche_demolished",
]


def execute_command_per_mission(cmd, base, missions, parallel=1):
    summary = {}
    for j, mission_name in enumerate(missions):
        p = os.path.join(base, mission_name)
        env = {"MISSION_DATA": str(p)}
        try:
            cmd_out = cmd
            if (j - parallel + 1) % parallel != 0:  # TRUST ME BRO
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
    for k, v in summary.items():
        print(k, ": ", v)


cmd = "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/verification/evo_evaluation.py --config_name=evo_modality_local"
# cmd = "python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/color_correction.py"
execute_command_per_mission(cmd, base, missions, parallel=1)
