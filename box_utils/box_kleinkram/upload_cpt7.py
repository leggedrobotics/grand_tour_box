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
        file_keys = [
            "_cpt7_ie_ppptc.bag",
            "_cpt7_ie_dgps.bag",
            "_cpt7_ie_ppp.bag",
            "_cpt7_ie_tc.bag",
            "_cpt7_ie_lc.bag",
        ]  # "_cpt7_raw_imu.bag",
        file_paths = []
        for file_key in file_keys:
            try:
                file_paths.append(get_bag(f"*{file_key}"))
                mission_name = "pub_" + file_paths[-1].split("/")[-1].replace(file_key, "")
            except Exception:
                pass

        if len(file_paths) == 0:
            exit(99)

        for file_path in file_paths:
            upload_simple(project_name="GrandTour", mission_name=mission_name, path=file_path)

        if len(file_paths) == 1:
            exit(1)
        if len(file_paths) == 2:
            exit(2)
        if len(file_paths) == 3:
            exit(3)
        if len(file_paths) == 4:
            exit(4)

        exit(0)
