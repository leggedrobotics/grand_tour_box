import argparse
import os
from pathlib import Path


def remove(data_folder, mission_name):
    MISSION_DATA = os.path.join(data_folder, mission_name)
    keys = [
        "_jetson_zed2i_tf.bag",
        "_jetson_zed2i_prop.bag",
        "_jetson_zed2i_images.bag",
        "_jetson_zed2i_depth.bag",
        "_jetson_hdr_right.bag",
        "_jetson_hdr_left.bag",
        "_jetson_hdr_front.bag",
    ]
    folder = Path(MISSION_DATA)
    files_to_upload = [str(s) for s in folder.rglob("*.bag")]

    for f in files_to_upload:
        for k in keys:
            if k in f:
                os.system(f"rm {f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process and upload mission data.")
    parser.add_argument("--data_folder", default="/data", help="Base path")
    parser.add_argument("--mission_names", nargs="+", help="One or more mission names to process.")

    args = parser.parse_args()

    for mission_name in args.mission_names:
        remove(args.data_folder, mission_name)
