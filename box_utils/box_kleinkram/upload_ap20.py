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
        path_robot = get_bag("*_jetson_ap20_robot.bag")
        path_synced = get_bag("*_jetson_ap20_synced.bag")
        mission_name = "pub_" + path_robot.split("/")[-1].replace("_jetson_ap20_robot.bag", "")

        upload_simple(project_name="GrandTour", mission_name=mission_name, path=path_robot)
        upload_simple(project_name="GrandTour", mission_name=mission_name, path=path_synced)
