from box_auto.utils import get_bag, upload_bag, run_ros_command, kill_roscore

if __name__ == "__main__":
    bags = []
    try:
        bags.append(get_bag("*nuc_livox.bag"))
    except:
        pass
    try:
        bags.append(get_bag("*nuc_hesai_post_processed.bag"))
    except:
        pass
    try:
        bags.append(get_bag("*npc_velodyne_processed.bag"))
    except:
        pass

    kill_roscore()
    out_bags = []
    for bag in bags:
        if "velodyne" in bag:
            out_bag = bag.replace("npc_velodyne_processed.bag", "npc_velodyne_ready.bag")
        elif "hesai" in bag:
            out_bag = bag.replace("nuc_hesai_post_processed.bag", "nuc_hesai_ready.bag")
        elif "livox" in bag:
            out_bag = bag.replace("nuc_livox.bag", "nuc_livox_ready.bag")
        else:
            out_bag = bag.replace(".bag", "_filtered.bag")

        out_bags.append(out_bag)
        run_ros_command(
            f"roslaunch box_auto box_filter_lidars.launch global_input_bag_path:={bag} global_output_bag_path:={out_bag}"
        )
    kill_roscore()
    upload_bag([out_bags])
