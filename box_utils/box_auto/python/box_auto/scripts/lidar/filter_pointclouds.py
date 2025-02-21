from box_auto.utils import get_bag, upload_bag, run_ros_command, kill_roscore

if __name__ == "__main__":
    bags = []
    bags.append(get_bag("*nuc_livox.bag"))
    bags.append(get_bag("*nuc_hesai_post_processed.bag"))

    kill_roscore()
    out_bags = []
    for bag in bags:

        out_bag = bag.replace(".bag", "_filtered.bag")
        out_bags.append(out_bag)
        run_ros_command(
            f"roslaunch box_auto box_filter_lidars.launch global_input_bag_path:={bag} global_output_bag_path:={out_bag}"
        )
    kill_roscore()

    upload_bag([out_bags[0], out_bags[1]])
