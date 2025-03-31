from box_auto.utils import get_bag, execute_command_per_mission
import os
import argparse
import kleinkram
from pathlib import Path

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
        print("verify", mission_name)

        res = kleinkram.verify(project_name="GrandTour", mission_name=mission_name, files=[path])
        print(res)

        if res[Path(path)] != "matched":
            fileinfo = [
                f
                for f in kleinkram.list_files(project_names=["GrandTour"], mission_names=[mission_name])
                if Path(path).name in f.name
            ]

            print(fileinfo[0])

            if fileinfo[0].state != "OK":
                print("Maybe delte file")

            else:
                print("File is OK")
        else:
            print("Hash matched")

        os.system(f"klein upload --project GrandTour --mission {mission_name} {path}")
