########################################################################
#
# Standalone script to check the frequency of the data in an SVO file.
#
# Note: This can only be run on the Jetson, which has the Zed2i sdk installed.
#
########################################################################

"""
    Read in an SVO file and calculate the frequency of the data, printing it to the console.
"""

import pyzed.sl as sl
import argparse
import os
import numpy as np


def calculate_moving_average_fps(timestamps, window_size=10):
    fps_values = []
    for i in range(len(timestamps) - window_size):
        start_time = timestamps[i]
        end_time = timestamps[i + window_size]
        duration_ns = end_time - start_time
        duration_s = duration_ns / 1e9
        fps = window_size / duration_s
        fps_values.append(fps)
    return fps_values


def main(filepath):
    print("\nOpening SVO file:", filepath)
    input_type = sl.InputType()
    input_type.set_from_svo_file(filepath)  # Set init parameter to run from the .svo
    init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    init.depth_mode = sl.DEPTH_MODE.NONE
    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:  # Ensure the camera opened successfully
        print("Camera Open", status, "Exit program.")
        exit(1)

    nb_frames = cam.get_svo_number_of_frames()
    print("[Info] SVO contains", nb_frames, "frames")

    timestamps = []

    print("Calculating FPS, can take a while (iterating through all frames)...")
    runtime = sl.RuntimeParameters()
    while cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        svo_position = cam.get_svo_position()
        if svo_position % nb_frames // 10 == 0:
            print(f"Progress: {(svo_position/nb_frames)*100}%")
        if svo_position >= nb_frames - 1:
            break
        timestamp = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        timestamps.append(timestamp)

    if len(timestamps) > 10:
        fps_values = calculate_moving_average_fps(timestamps)
        min_fps = min(fps_values)
        max_fps = max(fps_values)
        mean_fps = np.mean(fps_values)
        print(f"Minimum FPS (moving average over 10 frames): {min_fps:.2f} Hz")
        print(f"Maximum FPS (moving average over 10 frames): {max_fps:.2f} Hz")
        print(f"Mean FPS (moving average over 10 frames): {mean_fps:.2f} Hz")
    else:
        print("Not enough frames to calculate moving average.")

    cam.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_svo_file", type=str, help="Path to the SVO file", required=True)
    args = parser.parse_args()

    if not args.input_svo_file.endswith(".svo") and not args.input_svo_file.endswith(".svo2"):
        print("--input_svo_file parameter should be a .svo file but is not:", args.input_svo_file, "Exit program.")
        exit()
    if not os.path.isfile(args.input_svo_file):
        print("--input_svo_file parameter should be an existing file but is not:", args.input_svo_file, "Exit program.")
        exit()

    main(args.input_svo_file)
