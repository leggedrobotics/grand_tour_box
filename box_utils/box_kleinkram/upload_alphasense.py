from box_auto.utils import get_bag, execute_command_per_mission
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
        path = get_bag("*_nuc_alphasense_color.bag")
        mission_name = "pub_" + path.split("/")[-1].replace("_nuc_alphasense_color.bag", "")
        os.system(f"klein upload --project GrandTour --mission {mission_name} {path}")
