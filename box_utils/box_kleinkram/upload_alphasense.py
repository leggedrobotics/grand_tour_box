from box_auto.utils import get_bag, execute_command_per_mission, upload_simple
import os
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", default=False, action="store_true")

    args = parser.parse_args()

    if args.all:
        # Call the same script but for each mission
        cmd = "python3 " + str(os.path.abspath(__file__))
        execute_command_per_mission(cmd)

    else:
        # Upload the data
        k = "_nuc_alphasense_color.bag"
        path = get_bag(f"*{k}")
        mission_name = "pub_" + path.split("/")[-1].replace(k, "")

        print("Mission:", mission_name)
        upload_simple(project_name="GrandTour", mission_name=mission_name, path=path)
    print("", flush=True)
