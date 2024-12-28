#!/usr/bin/env python3
from box_auto.utils import get_file, MISSION_DATA, run_ros_command


if __name__ == "__main__":
    # Find relevant files
    zed_svos = get_file("*_jetson_zed2i.svo2", MISSION_DATA)

    # Process ZED 2I SVO2 file
    for zed_svo in zed_svos:
        print(f"Processing ZED SVO2 file: {zed_svo}")
        return_code = run_ros_command(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")

        if return_code == 0:
            print(f"Successfully processed {zed_svo}")
        else:
            print(f"Error processing {zed_svo} (Return code: {return_code})")

        _, depth_bag_exists = get_file("*_jetson_zed2i_depth.bag")
        _, image_bag_exists = get_file("*_jetson_zed2i_images.bag")
        _, prop_bag_exists  = get_file("*_jetson_zed2i_prop.bag")
        _, tf_bag_exists    = get_file("*_jetson_zed2i_tf.bag")

        if not depth_bag_exists:
            print("Depth bag does not exist!")
            exit(-1)

        if not image_bag_exists:
            print("Image bag does not exist!")
            exit(-1)

        if not prop_bag_exists:
            print("Prop bag does not exist!")
            exit(-1)

        if not tf_bag_exists:
            print("TF bag does not exist!")
            exit(-1)

        print("Auto terminating.")        

        exit(0)
