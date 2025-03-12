#!/usr/bin/env python3
from box_auto.utils import get_file, get_bag, run_ros_command, kill_roscore

if __name__ == "__main__":
    # Find input file
    # Expected 1 per mission.
    zed_svos = get_file("*_jetson_zed2i.svo2")

    # Ensure no roscore is active
    kill_roscore()

    # Process ZED2i SVO2 file
    for zed_svo in zed_svos:
        print(f"Processing ZED SVO2 file: {zed_svo}")
        return_code = run_ros_command(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")

        if return_code == 0:
            print(f"Successfully processed {zed_svo} file.")
        else:
            kill_roscore()
            print(f"Error processing {zed_svo} (Return code: {return_code})")

        # Expected output patterns
        output_patterns = [
            "*_jetson_zed2i_depth.bag",
            "*_jetson_zed2i_images.bag",
            "*_jetson_zed2i_prop.bag",
            "*_jetson_zed2i_tf.bag",
        ]

        # Iterate through each output pattern to ensure it is located where its expected.
        for pattern in output_patterns:
            get_bag(pattern=pattern, auto_download=False)

        print("ZED2i automation script is successfully finished. Self-terminating.")

        exit(0)
