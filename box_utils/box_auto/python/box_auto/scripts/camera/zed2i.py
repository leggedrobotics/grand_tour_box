#!/usr/bin/env python3
from box_auto.utils import get_file, run_ros_command, kill_roscore

if __name__ == "__main__":
    # Find input file
    zed_svos = get_file("*_jetson_zed2i.svo2")
    # zed_svos = get_file("*_jetson_zed2i.svo2",rglob=True, return_list=True)

    # Ensure no roscore is active
    kill_roscore()

    # Process ZED 2I SVO2 file
    for zed_svo in zed_svos:
        print(f"Processing ZED SVO2 file: {zed_svo}")
        return_code = run_ros_command(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")

        if return_code == 0:
            print(f"Successfully processed {zed_svo} file.")
        else:
            kill_roscore()
            print(f"Error processing {zed_svo} (Return code: {return_code})")

    kill_roscore()
    print("ZED2i automation script is successfully finished. Self-terminating.")

    exit(0)
