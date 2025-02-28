from box_auto.utils import get_bag, upload_bag, run_ros_command

if __name__ == "__main__":

    # Get the bag paths
    livox_bag = get_bag("*nuc_livox.bag")
    hesai_bag = get_bag("*nuc_hesai_post_processed.bag")

    # Get the output bag paths
    livox_out_bag = livox_bag.replace("nuc_livox.bag", "nuc_livox_filtered.bag")
    hesai_out_bag = hesai_bag.replace("nuc_hesai_post_processed.bag", "nuc_hesai_filtered.bag")

    # Run for hesai
    operation_mode = "hesai"
    run_ros_command(
        f"roslaunch box_auto box_filter_lidars.launch operation_mode:={operation_mode} global_input_bag_path:={hesai_bag} global_output_bag_path:={hesai_out_bag}"
    )

    # Run for livox
    operation_mode = "livox"
    run_ros_command(
        f"roslaunch box_auto box_filter_lidars.launch operation_mode:={operation_mode} global_input_bag_path:={livox_bag} global_output_bag_path:={livox_out_bag}"
    )

    upload_bag([livox_out_bag, hesai_out_bag])
